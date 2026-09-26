//! The file-open fallback for WSL: when `dotv` runs inside WSL, the platform
//! file dialog is the XDG file-chooser portal, which has no session D-Bus to
//! talk to under WSLg (most WSL distros run no `xdg-desktop-portal` and no
//! user bus at `/run/user/<uid>/bus`), so `prompt_for_paths` fails and no
//! file browser ever opens.
//!
//! WSL files are a Windows network share, though: `/home/dwpeng` on Debian is
//! `\\wsl.localhost\Debian\home\dwpeng` on Windows — the same location that
//! typing `\\wsl$` into Explorer's address bar reaches. So the fallback asks
//! Windows for the file: it runs the Explorer common dialog through
//! `powershell.exe` (WSL interop), starts it in the current folder as Windows
//! sees it, and translates the picked Windows path back to its native Linux
//! path with `wslpath`, so the file is read directly from the Linux
//! filesystem instead of over the 9P network share.

use std::path::{Path, PathBuf};
use std::process::Command;

/// True when the process runs inside WSL. `WSL_DISTRO_NAME` / `WSL_INTEROP`
/// are set by WSL itself; the kernel-release check covers runs where the
/// environment was scrubbed (e.g. a launch through `env -i`).
pub fn running_in_wsl() -> bool {
    if std::env::var_os("WSL_DISTRO_NAME").is_some() || std::env::var_os("WSL_INTEROP").is_some() {
        return true;
    }
    std::fs::read_to_string("/proc/sys/kernel/osrelease")
        .map(|release| release.to_ascii_lowercase().contains("microsoft"))
        .unwrap_or(false)
}

/// Opens the Windows file dialog through WSL interop and returns the picked
/// file as a native Linux path. `start_dir` is the Linux directory the
/// dialog should open in. `Ok(None)` means the user closed the dialog
/// without picking; `Err` means the Windows side could not be reached at
/// all, and the caller should fall back to the platform dialog.
pub fn pick_file(title: &str, start_dir: Option<&Path>) -> Result<Option<PathBuf>, String> {
    let powershell = find_powershell()
        .ok_or_else(|| "powershell.exe not found: WSL interop is disabled".to_string())?;

    // Open the dialog in the WSL folder as Windows sees it —
    // `\\wsl.localhost\<distro>\home\...` — falling back to the WSL home.
    // `wslpath -w` only names existing paths, so walk up to the nearest
    // ancestor that does.
    let initial_dir = start_dir
        .map(existing_ancestor)
        .as_deref()
        .and_then(windows_path)
        .or_else(|| {
            std::env::var_os("HOME")
                .map(PathBuf::from)
                .as_deref()
                .and_then(windows_path)
        });

    let mut script = String::from(
        "[Console]::OutputEncoding = [System.Text.Encoding]::UTF8; \
         Add-Type -AssemblyName System.Windows.Forms | Out-Null; \
         $d = New-Object System.Windows.Forms.OpenFileDialog; \
         $d.Title = ",
    );
    script.push_str(&ps_quote(title));
    script.push_str("; $d.Filter = 'DOT files (*.dot;*.gv)|*.dot;*.gv|All files (*.*)|*.*';");
    if let Some(initial_dir) = &initial_dir {
        script.push_str(" $d.InitialDirectory = ");
        script.push_str(&ps_quote(initial_dir));
        script.push(';');
    }
    script.push_str(
        " if ($d.ShowDialog() -eq [System.Windows.Forms.DialogResult]::OK) \
         { [Console]::Out.Write($d.FileName) }",
    );

    let output = Command::new(&powershell)
        .args([
            "-NoProfile",
            "-NonInteractive",
            "-STA",
            "-WindowStyle",
            "Hidden",
            "-Command",
            &script,
        ])
        .output()
        .map_err(|err| format!("failed to launch powershell.exe: {err}"))?;

    let picked = String::from_utf8_lossy(&output.stdout);
    let picked = picked.trim_matches(|c: char| c.is_whitespace() || c == '\u{feff}');
    if picked.is_empty() {
        if output.status.success() {
            return Ok(None); // the dialog was closed without a pick
        }
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Windows file dialog failed: {}", stderr.trim()));
    }
    Ok(Some(to_linux_path(picked)))
}

/// Locates `powershell.exe`: via `PATH` (WSL appends the Windows system
/// paths when interop is on), then the well-known absolute location.
fn find_powershell() -> Option<PathBuf> {
    const CANDIDATES: [&str; 2] = [
        "powershell.exe",
        "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe",
    ];
    CANDIDATES.into_iter().map(PathBuf::from).find(|candidate| {
        if candidate.is_absolute() {
            candidate.is_file()
        } else {
            on_path(candidate)
        }
    })
}

fn on_path(name: &Path) -> bool {
    std::env::var_os("PATH")
        .map(|search| std::env::split_paths(&search).any(|dir| dir.join(name).is_file()))
        .unwrap_or(false)
}

/// The nearest ancestor of `dir` that exists, so `wslpath -w` (which
/// requires an existing path) can convert it.
fn existing_ancestor(dir: &Path) -> PathBuf {
    let mut dir = dir.to_path_buf();
    loop {
        match (dir.exists(), dir.parent()) {
            (true, _) | (_, None) => return dir,
            (_, Some(parent)) => dir = parent.to_path_buf(),
        }
    }
}

/// `path` as Windows sees it: `/home/dwpeng` becomes
/// `\\wsl.localhost\Debian\home\dwpeng`. `None` when Windows cannot name the
/// path (another distro's share, a drive that is not mounted), which the
/// caller treats as "no start directory".
fn windows_path(path: &Path) -> Option<String> {
    let output = Command::new("wslpath").arg("-w").arg(path).output().ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&output.stdout).trim().to_string();
    (!text.is_empty()).then_some(text)
}

/// Quotes a string for PowerShell: a single-quoted literal, with embedded
/// quotes doubled.
fn ps_quote(text: &str) -> String {
    format!("'{}'", text.replace('\'', "''"))
}

/// Parses pasted text into candidate file paths. Copying a file in Windows
/// Explorer puts its path (or paths, one per line) on the clipboard as text —
/// the format WSLg actually forwards into Linux — so each line is trimmed,
/// unwrapped from quotes and `file://` URIs, and translated from a Windows
/// path to its native Linux path where possible.
pub fn parse_pasted_paths(text: &str) -> Vec<PathBuf> {
    let mut paths = Vec::new();
    for line in text.lines() {
        // `file:///path` is a local file URI: the third slash starts the path.
        let line = match line.trim().strip_prefix("file://") {
            Some(rest) if rest.starts_with('/') => rest,
            Some(_) => continue, // remote or malformed URI: not a local file
            None => line.trim(),
        };
        let line = line.trim_matches(|c: char| c == '"' || c == '\'').trim();
        if line.is_empty() {
            continue;
        }
        let path = to_linux_path(line);
        if !paths.contains(&path) {
            paths.push(path);
        }
    }
    paths
}

/// The native Linux path for a Windows-picked path. `wslpath -u` knows the
/// drive mounts and the `\\wsl.localhost\<distro>\...` share; the manual
/// strip below covers runs where `wslpath` is missing or refuses the shape.
fn to_linux_path(windows: &str) -> PathBuf {
    if let Ok(output) = Command::new("wslpath").arg("-u").arg(windows).output()
        && output.status.success() {
            let text = String::from_utf8_lossy(&output.stdout);
            let text = text.trim_matches(|c: char| c.is_whitespace() || c == '\u{feff}');
            if text.starts_with('/') {
                return PathBuf::from(text);
            }
        }
    // `\\wsl.localhost\Debian\home\...` / `\\wsl$\Debian\home\...` → `/home/...`
    let normalized = windows.replace('\\', "/");
    let lower = normalized.to_ascii_lowercase();
    for prefix in ["//wsl.localhost/", "//wsl$/"] {
        if lower.starts_with(prefix)
            && let Some((_distro, tail)) = normalized[prefix.len()..].split_once('/')
        {
            return PathBuf::from(format!("/{tail}"));
        }
    }
    PathBuf::from(windows)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ps_quote_escapes_embedded_quotes() {
        assert_eq!(ps_quote("select 'x'"), "'select ''x'''");
        assert_eq!(ps_quote("plain"), "'plain'");
    }

    #[test]
    fn wsl_share_paths_map_back_to_linux() {
        // On WSL `wslpath` does the mapping (same result); elsewhere the
        // manual strip below does.
        assert_eq!(
            to_linux_path(r"\\wsl.localhost\Debian\home\dwpeng\graph.dot"),
            PathBuf::from("/home/dwpeng/graph.dot")
        );
        assert_eq!(
            to_linux_path(r"\\wsl$\Debian\home\dwpeng\graph.dot"),
            PathBuf::from("/home/dwpeng/graph.dot")
        );
    }

    #[test]
    fn pasted_text_yields_one_path_per_line() {
        assert_eq!(
            parse_pasted_paths("/tmp/a.dot\n\n/tmp/b.gv\n"),
            vec![PathBuf::from("/tmp/a.dot"), PathBuf::from("/tmp/b.gv"),]
        );
        // Explorer wraps a single copied file's path in quotes; the Windows
        // path itself is translated by `wslpath`, which only exists under
        // WSL, so assert the unwrapping on a platform-independent path.
        assert_eq!(
            parse_pasted_paths("\"/tmp/my graph.dot\""),
            vec![PathBuf::from("/tmp/my graph.dot")]
        );
        assert_eq!(
            parse_pasted_paths("file:///home/dwpeng/graph.dot"),
            vec![PathBuf::from("/home/dwpeng/graph.dot")]
        );
        assert!(parse_pasted_paths("   ").is_empty());
    }
}

//! Where the settings live on disk: a `settings.json` under the user's config
//! directory, written atomically and read defensively.
//!
//! Persistence is a convenience, never a precondition. A file that is missing,
//! unreadable, stale, hand-edited into a typo, or a config directory the
//! process may not write to all end the same way: the viewer starts, on
//! whatever it could make sense of, and carries on.

use std::ffi::OsString;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use super::Settings;

/// The directory the settings file sits in, under the platform's config root.
const APP_DIR: &str = "dotv";
const FILE_NAME: &str = "settings.json";

/// A settings file on disk.
#[derive(Clone, Debug)]
pub struct SettingsStore {
    path: PathBuf,
}

impl SettingsStore {
    /// The store in the user's config directory —
    /// `~/.config/dotv/settings.json` on Linux, `~/Library/Application
    /// Support/dotv/…` on macOS, `%APPDATA%\dotv\…` on Windows.
    ///
    /// `None` when the platform offers no config directory at all; the viewer
    /// then runs on its defaults and writes nothing.
    pub fn user() -> Option<Self> {
        config_dir().map(|root| Self::at(root.join(APP_DIR).join(FILE_NAME)))
    }

    /// A store at an explicit path.
    pub fn at(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }

    /// The file this store reads and writes.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Reads the settings, or the defaults when the file is not there or cannot
    /// be made sense of (see [`Settings::from_json`]).
    pub fn load(&self) -> Settings {
        match fs::read_to_string(&self.path) {
            Ok(text) => Settings::from_json(&text),
            Err(_) => Settings::default(),
        }
    }

    /// Writes the settings, creating the directory if it is missing.
    ///
    /// The text goes to a temporary file beside the target, which is then
    /// renamed over it. That matters because the reader treats an unparsable
    /// file as "all defaults": a write interrupted halfway would otherwise
    /// quietly reset every setting the next time the viewer started.
    pub fn save(&self, settings: &Settings) -> io::Result<()> {
        if let Some(dir) = self.path.parent() {
            fs::create_dir_all(dir)?;
        }
        let temp = self.path.with_extension("json.tmp");
        fs::write(&temp, settings.to_json())?;
        fs::rename(&temp, &self.path)
    }
}

/// The platform's per-user config root, by that platform's own convention.
fn config_dir() -> Option<PathBuf> {
    if cfg!(target_os = "windows") {
        // Roaming, not Local: settings are meant to follow the user.
        return non_empty(std::env::var_os("APPDATA")).map(PathBuf::from);
    }
    if cfg!(target_os = "macos") {
        return non_empty(std::env::var_os("HOME")).map(|home| {
            PathBuf::from(home)
                .join("Library")
                .join("Application Support")
        });
    }
    // The XDG base-directory spec: an absolute `XDG_CONFIG_HOME` wins, and a
    // relative one is to be ignored rather than resolved against the working
    // directory.
    if let Some(dir) = non_empty(std::env::var_os("XDG_CONFIG_HOME")).map(PathBuf::from)
        && dir.is_absolute()
    {
        return Some(dir);
    }
    non_empty(std::env::var_os("HOME")).map(|home| PathBuf::from(home).join(".config"))
}

/// `Some` for a variable that is set to something, `None` for unset or empty.
fn non_empty(value: Option<OsString>) -> Option<OsString> {
    value.filter(|value| !value.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_user_store_lands_in_the_config_directory() {
        let Some(store) = SettingsStore::user() else {
            return; // this environment has no config directory
        };
        assert!(
            store.path().ends_with("dotv/settings.json"),
            "{:?}",
            store.path()
        );
        assert!(store.path().is_absolute(), "{:?}", store.path());
    }

    #[test]
    fn saving_creates_the_directory_and_loading_reads_it_back() {
        let dir = std::env::temp_dir().join("dotv-settings-store-test");
        let path = dir.join("nested").join("settings.json");
        let _ = fs::remove_dir_all(&dir);
        let store = SettingsStore::at(&path);
        assert_eq!(
            store.load(),
            Settings::default(),
            "an absent file is default"
        );

        let settings = Settings {
            show_grid: false,
            label_scale: 1.4,
            ..Settings::default()
        };
        store
            .save(&settings)
            .expect("the write creates the directory");
        assert_eq!(store.load(), settings);
        assert!(
            !path.with_extension("json.tmp").exists(),
            "the temporary file is renamed over the target"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    /// A corrupt file must not stop the viewer, and must not stop the next save
    /// from replacing it.
    #[test]
    fn a_corrupt_file_is_replaced_rather_than_fatal() {
        let dir = std::env::temp_dir().join("dotv-settings-corrupt-test");
        let path = dir.join("settings.json");
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("create the directory");
        fs::write(&path, "{ this is not json").expect("write the corrupt file");

        let store = SettingsStore::at(&path);
        assert_eq!(store.load(), Settings::default());
        store
            .save(&Settings {
                show_grid: false,
                ..Settings::default()
            })
            .expect("overwrite it");
        assert!(!store.load().show_grid);
        let _ = fs::remove_dir_all(&dir);
    }
}

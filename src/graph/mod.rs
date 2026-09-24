//! The DOT data layer: reading a DOT source into the [`Graph`] structure.
//! Pure data — no geometry, no rendering.

pub mod model;
pub mod parser;

use std::fmt;
use std::path::PathBuf;

pub use model::Graph;
pub use parser::DotError;

/// Why a DOT file could not be shown.
#[derive(Debug)]
pub enum LoadError {
    Io(std::io::Error),
    Parse(DotError),
    /// The file parsed, but the layout engine produced a drawing that cannot
    /// be rendered (non-finite geometry, missing nodes, …).
    Layout(String),
}

impl fmt::Display for LoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LoadError::Io(err) => write!(f, "Failed to read file: {err}"),
            LoadError::Parse(err) => write!(f, "Failed to parse DOT: {err}"),
            LoadError::Layout(why) => {
                write!(f, "The layout engine produced an invalid drawing: {why}")
            }
        }
    }
}

impl std::error::Error for LoadError {}

/// Reads a DOT file from disk and parses it into a [`Graph`]. Returns the
/// canonicalized-on-arrival path (as given) together with the graph, so
/// callers can keep both without re-reading.
pub fn load(path: impl Into<PathBuf>) -> Result<(PathBuf, Graph), LoadError> {
    let path = path.into();
    let source = std::fs::read_to_string(&path).map_err(LoadError::Io)?;
    let graph = parser::parse(&source).map_err(LoadError::Parse)?;
    Ok((path, graph))
}

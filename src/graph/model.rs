//! The domain model for a parsed DOT graph — a faithful mirror of the
//! [cgraph](https://gitlab.com/graphviz/graphviz/-/tree/main/lib/cgraph)
//! data model that the dot engine consumes.
//!
//! One [`Graph`] owns a flat node table and a flat edge table (cgraph's root
//! graph), plus a tree of [`Subgraph`]s — cgraph's `Agraph_t` objects, which
//! are both "subgraphs" (node/edge membership + their own graph attributes,
//! e.g. `rank=same`) and, when their name starts with `cluster`, the layout
//! clusters dot draws boxes around. Membership mirrors cgraph exactly:
//!
//! - a node declared inside a subgraph statement becomes a member of that
//!   subgraph (and of the root, which owns it);
//! - an edge declared inside a subgraph becomes a member of exactly that
//!   subgraph;
//! - iteration order everywhere is declaration (first-use) order, which the
//!   dot pipeline's tie-breaking relies on.

use std::collections::{BTreeMap, HashMap, HashSet};

/// An ordered map of DOT attribute names to values. Keys are lowercased
/// (DOT attribute names are case-insensitive).
pub type Attributes = BTreeMap<String, String>;

/// Whether the graph is directed (`digraph`, `->`) or undirected (`graph`, `--`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GraphKind {
    #[default]
    Directed,
    Undirected,
}

/// A node or edge port as written in the source (`a:p`, `a:nw`, `a:p:ne`).
///
/// Kept raw — exactly the string cgraph stores in the `tailport`/`headport`
/// edge attributes (`"p"`, `"p:n"`, …) — and interpreted later, when the
/// shape knows how to resolve record/HTML port ids and compass points.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Port {
    pub raw: String,
}

impl Port {
    pub fn new(raw: impl Into<String>) -> Self {
        Self { raw: raw.into() }
    }
}

/// A node in the graph, indexed by its position in [`Graph::nodes`].
#[derive(Debug, Clone)]
pub struct Node {
    /// The DOT id of the node. In scope while the source document is loaded.
    pub name: String,
    /// Attribute values already merged with the `node[...]` defaults.
    pub attrs: Attributes,
    /// Names of attributes whose value was an HTML-like string (`<...>`).
    /// Only `label`/`xlabel`/`head_label`/`tail_label` are meaningful here.
    pub html_attrs: HashSet<String>,
}

impl Node {
    /// The display label: the `label` attribute when present, else the name.
    pub fn label(&self) -> String {
        self.attrs
            .get("label")
            .filter(|l| !l.is_empty())
            .cloned()
            .unwrap_or_else(|| self.name.clone())
    }

    /// True when the node's `label` was written as an HTML-like string.
    pub fn label_is_html(&self) -> bool {
        self.html_attrs.contains("label")
    }
}

/// One rendered line of a label — C's `textspan_t` (justification + text).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LabelLine {
    pub text: String,
    /// `'l'` (left), `'r'` (right) or `'n'` (centred) — from the escape that
    /// ended the line (`\l`, `\r`, or `\n`/a real newline).
    pub just: char,
}

/// `make_simple_label` (common/labels.c:57-105) — split a label's text into
/// lines at the `\n`/`\l`/`\r` escapes (and real newlines), resolving any
/// other `\x` escape to `x` (so `\\` collapses to `\`).
///
/// The escapes are *sizing* as well as layout: a three-line `\l` label is
/// three line-heights tall and as wide as its widest line, which is what
/// `poly_init` then pads into the node box.
pub fn label_lines(text: &str) -> Vec<LabelLine> {
    let mut lines: Vec<LabelLine> = Vec::new();
    let mut cur = String::new();
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\\' {
            match chars.next() {
                Some(j @ ('n' | 'l' | 'r')) => {
                    lines.push(LabelLine {
                        text: std::mem::take(&mut cur),
                        just: j,
                    });
                }
                // Drop the backslash, keep the escaped character (this is how
                // `\"` and `\\` are handled).
                Some(other) => cur.push(other),
                None => {}
            }
        } else if c == '\n' {
            lines.push(LabelLine {
                text: std::mem::take(&mut cur),
                just: 'n',
            });
        } else {
            cur.push(c);
        }
    }
    if !cur.is_empty() {
        lines.push(LabelLine {
            text: cur,
            just: 'n',
        });
    }
    lines
}

/// `strdup_and_subst_obj` (common/labels.c:387, escBackslash = 0) — resolve
/// the object escapes `\N`, `\G`, `\T`, `\H`, `\L`; every other escape
/// (including `\n`/`\l`/`\r` and `\\`) is left for [`label_lines`].
pub fn subst_label(text: &str, g: &str, n: &str, _e: &str, t: &str, h: &str, l: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.peek().copied() {
            Some('G') => {
                chars.next();
                out.push_str(g);
            }
            Some('N') => {
                chars.next();
                out.push_str(n);
            }
            Some('E') => {
                chars.next();
                // node -> node for a directed graph (undirected `--` is not
                // modelled here; dot is the only engine in this port).
                if !t.is_empty() || !h.is_empty() {
                    out.push_str(&format!("{t}->{h}"));
                }
            }
            Some('T') => {
                chars.next();
                out.push_str(t);
            }
            Some('H') => {
                chars.next();
                out.push_str(h);
            }
            Some('L') => {
                chars.next();
                out.push_str(l);
            }
            Some(other) => {
                // leave `\x` unmodified for `make_simple_label`
                out.push('\\');
                out.push(other);
                chars.next();
            }
            None => out.push('\\'),
        }
    }
    out
}

/// A directed or undirected edge between two nodes.
#[derive(Debug, Clone)]
pub struct Edge {
    /// Index into [`Graph::nodes`] of the tail node.
    pub tail: usize,
    /// Index into [`Graph::nodes`] of the head node.
    pub head: usize,
    /// Attribute values already merged with the `edge[...]` defaults.
    pub attrs: Attributes,
    /// Names of attributes whose value was an HTML-like string.
    pub html_attrs: HashSet<String>,
    /// The `:port` written after the tail node, if any.
    pub tail_port: Option<Port>,
    /// The `:port` written after the head node, if any.
    pub head_port: Option<Port>,
}

impl Edge {
    /// The display label of the edge, if the `label` attribute is set.
    pub fn label(&self) -> Option<&str> {
        self.attrs
            .get("label")
            .filter(|l| !l.is_empty())
            .map(String::as_str)
    }

    /// True when the edge's `label` was written as an HTML-like string.
    pub fn label_is_html(&self) -> bool {
        self.html_attrs.contains("label")
    }
}

/// A cgraph subgraph — a membership set with its own graph attributes.
///
/// One of these is also a layout *cluster* when [`Self::is_cluster`] holds.
#[derive(Debug, Clone)]
pub struct Subgraph {
    /// The subgraph name (anonymous subgraphs get a generated one; cgraph
    /// names them internally and the name is irrelevant for layout).
    pub name: String,
    /// `true` for the root graph, for names starting with `cluster`
    /// (case-insensitive), or when its `cluster` attribute is true —
    /// `is_a_cluster()` in `lib/common/utils.c`.
    pub is_cluster: bool,
    /// Graph attributes set on this subgraph (`rank=same`, `label`, ...).
    pub attrs: Attributes,
    /// Names of attributes whose value was an HTML-like string.
    pub html_attrs: HashSet<String>,
    /// Member nodes in first-membership order.
    pub nodes: Vec<usize>,
    /// Member edges in declaration order.
    pub edges: Vec<usize>,
    /// Child subgraphs in creation order.
    pub children: Vec<usize>,
}

/// A parsed DOT graph.
#[derive(Debug, Clone, Default)]
pub struct Graph {
    pub kind: GraphKind,
    pub strict: bool,
    /// The optional graph name (`digraph G { .. }`).
    pub name: Option<String>,
    /// All nodes in first-use order (the root graph's node iteration order).
    pub nodes: Vec<Node>,
    /// All edges in declaration order.
    pub edges: Vec<Edge>,
    /// Subgraph tree; index 0 is the root graph itself.
    pub subgraphs: Vec<Subgraph>,
    /// Defaults from `node[...]` statements (root-level, like cgraph's
    /// root-scoped attribute defaults).
    pub default_node_attrs: Attributes,
    /// Defaults from `edge[...]` statements.
    pub default_edge_attrs: Attributes,
    node_index: HashMap<String, usize>,
}

impl Graph {
    /// Graph attributes of the root graph (`rankdir`, ...).
    pub fn graph_attrs(&self) -> &Attributes {
        &self.subgraphs[0].attrs
    }

    /// Sets a root graph attribute (used by the settings-driven relayout).
    pub fn set_graph_attr(&mut self, key: &str, value: &str) {
        self.subgraphs[0].attrs.insert(key.into(), value.into());
    }

    /// Returns the index of the node with `name`, creating it with the
    /// current `node[...]` defaults merged in when it does not exist yet.
    pub fn ensure_node(&mut self, name: &str) -> usize {
        if let Some(&index) = self.node_index.get(name) {
            return index;
        }
        let index = self.nodes.len();
        self.nodes.push(Node {
            name: name.to_string(),
            attrs: self.default_node_attrs.clone(),
            html_attrs: HashSet::new(),
        });
        self.node_index.insert(name.to_string(), index);
        index
    }

    /// Adds a node to a subgraph's membership if not already a member.
    pub fn subgraph_add_node(&mut self, sg: usize, node: usize) {
        if !self.subgraphs[sg].nodes.contains(&node) {
            self.subgraphs[sg].nodes.push(node);
        }
    }

    /// Adds an edge to a subgraph's membership if not already a member.
    pub fn subgraph_add_edge(&mut self, sg: usize, edge: usize) {
        if !self.subgraphs[sg].edges.contains(&edge) {
            self.subgraphs[sg].edges.push(edge);
        }
    }

    /// Finds a child subgraph of `parent` by name, if it exists
    /// (cgraph's `agsubg(g, name, 0)` lookup semantics).
    pub fn find_subgraph(&self, parent: usize, name: &str) -> Option<usize> {
        self.subgraphs[parent]
            .children
            .iter()
            .copied()
            .find(|&sg| self.subgraphs[sg].name == name)
    }

    /// The node's label, escape-substituted and split into lines
    /// (`make_simple_label`): `\N` is the node's name, `\G` the graph name.
    pub fn node_label_lines(&self, index: usize) -> Vec<LabelLine> {
        let node = &self.nodes[index];
        let g = self.name.clone().unwrap_or_default();
        let raw = node.label();
        label_lines(&subst_label(&raw, &g, &node.name, "", "", "", &raw))
    }

    /// The edge's label lines, or `None` when the edge has no label.
    pub fn edge_label_lines(&self, index: usize) -> Option<Vec<LabelLine>> {
        let edge = &self.edges[index];
        let raw = edge.label()?;
        let g = self.name.clone().unwrap_or_default();
        let t = &self.nodes[edge.tail].name;
        let h = &self.nodes[edge.head].name;
        Some(label_lines(&subst_label(raw, &g, "", "", t, h, raw)))
    }

    /// True when the subgraph's `label` was written as an HTML-like string.
    pub fn label_is_html(&self, sg: usize) -> bool {
        self.subgraphs[sg].html_attrs.contains("label")
    }

    /// A subgraph's (cluster's) label lines.
    pub fn subgraph_label_lines(&self, sg: usize) -> Option<Vec<LabelLine>> {
        let raw = self.subgraphs[sg]
            .attrs
            .get("label")
            .filter(|l| !l.is_empty())?;
        let g = self.name.clone().unwrap_or_default();
        Some(label_lines(&subst_label(raw, &g, "", "", "", "", raw)))
    }
}

#[cfg(test)]
mod label_tests {
    use super::*;

    /// `make_simple_label` (common/labels.c:57-105): `\l`/`\r` terminate a
    /// line and set its justification, a real newline terminates with `'n'`,
    /// and every other escape loses its backslash (`\\` -> `\`, `\N` -> `N`).
    #[test]
    fn label_lines_split_and_justify() {
        let lines = label_lines(r"one\ltwo\rthree\nfour\N");
        // The trailing partial line keeps the C default justification.
        assert_eq!(
            lines
                .iter()
                .map(|l| (l.text.as_str(), l.just))
                .collect::<Vec<_>>(),
            vec![("one", 'l'), ("two", 'r'), ("three", 'n'), ("fourN", 'n')]
        );
        // An empty line is still a line.
        let lines = label_lines("a\n\nb");
        assert_eq!(lines.len(), 3);
        assert!(lines[1].text.is_empty());
        // A label ending in a line terminator has no trailing empty line.
        assert_eq!(label_lines(r"a\l").len(), 1);
    }

    /// `strdup_and_subst_obj` resolves the object escapes; `\E` is
    /// `tail->head` for a directed edge.
    #[test]
    fn object_escapes_are_substituted() {
        assert_eq!(
            subst_label(r"\N in \G", "G", "n0", "", "", "", ""),
            "n0 in G"
        );
        assert_eq!(subst_label(r"\E", "G", "", "", "a", "b", ""), "a->b");
        assert_eq!(subst_label(r"\T \H", "G", "", "", "a", "b", ""), "a b");
        assert_eq!(subst_label(r"\L", "G", "", "", "", "", "lab"), "lab");
        // A doubled backslash escapes the substitution: with `escBackslash`
        // false the pair is left for `make_simple_label`, which collapses it.
        assert_eq!(subst_label(r"\\N", "G", "n0", "", "", "", ""), r"\\N");
        assert_eq!(label_lines(r"\\N")[0].text, r"\N");
        // Unknown escapes pass through for `make_simple_label`.
        assert_eq!(subst_label(r"\q", "G", "n0", "", "", "", ""), r"\q");
        // `\n` is a line break, not a substitution.
        assert_eq!(
            subst_label(r"one\ntwo", "G", "n0", "", "", "", ""),
            r"one\ntwo"
        );
    }
}

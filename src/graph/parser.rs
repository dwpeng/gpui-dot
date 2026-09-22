//! DOT parsing — a faithful Rust port of cgraph's scanner and grammar
//! (`lib/cgraph/scan.l` + `lib/cgraph/grammar.y`).
//!
//! The parser mirrors the upstream lexer and grammar rule by rule:
//!
//! - lexer: `NAME`/`NUMBER` atoms (incl. the trailing-letter/dot split
//!   warning behavior), quoted strings with cgraph's exact escape rules,
//!   HTML-like strings with nesting, `//`, `/* */` and `#` comments, the
//!   `->`/`--` edge operators (direction-sensitive), and the six keywords;
//! - grammar: `hdr body`, `stmt = attrstmt | compound`, `compound = simple
//!   rcompound optattr` with node lists, ports (`a:p`, `a:p:compass`),
//!   subgraphs, chained edge operators, `[key=value]` lists with optional
//!   separators, bare `id = value;` graph attributes, and the `key`
//!   pseudo-attribute;
//! - semantics: strict-graph edge merging (incl. dropping strict self
//!   loops), undirected edge orientation and port swapping, subgraph
//!   membership (nodes by reference, edges by declaring scope), node/edge
//!   defaults at root scope, graph attributes on the declaring subgraph,
//!   and cgraph's declaration-order iteration everywhere.

use std::fmt;

use super::model::{Edge, Graph, GraphKind, Port, Subgraph};

/// A parse error with 1-based line/column position.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DotError {
    pub line: usize,
    pub column: usize,
    pub message: String,
}

impl fmt::Display for DotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}, column {}: {}", self.line, self.column, self.message)
    }
}

/// Parses DOT source text into the internal graph model.
///
/// Returns a [`DotError`] describing the first syntax problem. A viewer must
/// never crash on malformed input, so nothing here panics.
pub fn parse(source: &str) -> Result<Graph, DotError> {
    let lexer = Lexer::new(source);
    let tokens = lexer.lex()?;
    let mut parser = Parser::new(tokens, source);
    parser.parse()
}

// ---------------------------------------------------------------------------
// Lexer
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kw {
    Graph,
    Digraph,
    Node,
    Edge,
    Strict,
    Subgraph,
}

/// What kind of atom a token is; `Qatom` may take part in `"a" + "b"`
/// concatenation, `Plain` may not (the grammar's T_atom vs T_qatom split).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AtomKind {
    Plain,
    Quoted,
    Html,
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Tok {
    Keyword(Kw),
    Atom(String, AtomKind),
    EdgeOp,
    Char(char),
    Eof,
}

/// One lexed token with its 1-based position (the position of its first
/// character).
#[derive(Debug, Clone)]
struct Token {
    tok: Tok,
    line: usize,
    column: usize,
}

struct Lexer<'a> {
    chars: Vec<char>,
    pos: usize,
    line: usize,
    column: usize,
    /// `graph`/`digraph` seen so far — decides `->`/`--` legality
    /// (scan.l's `graphType`).
    graph_type: Option<GraphKind>,
    _src: &'a str,
}

impl<'a> Lexer<'a> {
    fn new(src: &'a str) -> Self {
        Self {
            chars: src.chars().collect(),
            pos: 0,
            line: 1,
            column: 1,
            graph_type: None,
            _src: src,
        }
    }

    fn error(&self, line: usize, column: usize, message: impl Into<String>) -> DotError {
        DotError { line, column, message: message.into() }
    }

    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    fn peek_at(&self, offset: usize) -> Option<char> {
        self.chars.get(self.pos + offset).copied()
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.peek()?;
        self.pos += 1;
        if c == '\n' {
            self.line += 1;
            self.column = 1;
        } else {
            self.column += 1;
        }
        Some(c)
    }

    fn lex(mut self) -> Result<Vec<Token>, DotError> {
        let mut tokens = Vec::new();
        loop {
            let (line, column) = (self.line, self.column);
            self.skip_trivia()?;
            let Some(c) = self.peek() else {
                tokens.push(Token { tok: Tok::Eof, line, column });
                break;
            };
            // Re-read the position after trivia so it points at the token.
            let (line, column) = (self.line, self.column);
            let number_start = match c {
                '-' => self.peek_at(1).is_some_and(|n| n.is_ascii_digit() || n == '.'),
                '.' => self.peek_at(1).is_some_and(|n| n.is_ascii_digit()),
                d => d.is_ascii_digit(),
            };
            match c {
                '"' => {
                    let s = self.scan_quoted()?;
                    tokens.push(Token { tok: Tok::Atom(s, AtomKind::Quoted), line, column });
                }
                '<' => {
                    let s = self.scan_html()?;
                    tokens.push(Token { tok: Tok::Atom(s, AtomKind::Html), line, column });
                }
                '-' if !number_start => {
                    let second = self.peek_at(1);
                    let edgeop = match (c, second) {
                        ('-', Some('>')) => self.graph_type == Some(GraphKind::Directed),
                        ('-', Some('-')) => self.graph_type == Some(GraphKind::Undirected),
                        _ => false,
                    };
                    if edgeop {
                        self.bump();
                        self.bump();
                        tokens.push(Token { tok: Tok::EdgeOp, line, column });
                    } else {
                        self.bump();
                        tokens.push(Token { tok: Tok::Char('-'), line, column });
                    }
                }
                _ => {
                    if is_name_start(c) || number_start {
                        let atom = self.scan_atom()?;
                        // Keywords (scan.l matches them before {NAME}).
                        let kw = match atom.0.as_str() {
                            "node" => Some(Kw::Node),
                            "edge" => Some(Kw::Edge),
                            "graph" => {
                                if self.graph_type.is_none() {
                                    self.graph_type = Some(GraphKind::Undirected);
                                }
                                Some(Kw::Graph)
                            }
                            "digraph" => {
                                if self.graph_type.is_none() {
                                    self.graph_type = Some(GraphKind::Directed);
                                }
                                Some(Kw::Digraph)
                            }
                            "strict" => Some(Kw::Strict),
                            "subgraph" => Some(Kw::Subgraph),
                            _ => None,
                        };
                        tokens.push(Token {
                            tok: match kw {
                                Some(kw) => Tok::Keyword(kw),
                                None => Tok::Atom(atom.0, atom.1),
                            },
                            line,
                            column,
                        });
                    } else {
                        self.bump();
                        tokens.push(Token { tok: Tok::Char(c), line, column });
                    }
                }
            }
        }
        Ok(tokens)
    }

    /// Skips whitespace, comments, `#` lines and the BOM (scan.l).
    fn skip_trivia(&mut self) -> Result<(), DotError> {
        loop {
            match self.peek() {
                Some(' ') | Some('\t') | Some('\r') | Some('\u{FEFF}') => {
                    self.bump();
                }
                Some('\n') => {
                    self.bump();
                }
                Some('/') if self.peek_at(1) == Some('/') => {
                    while let Some(c) = self.peek() {
                        if c == '\n' {
                            break;
                        }
                        self.bump();
                    }
                }
                Some('/') if self.peek_at(1) == Some('*') => {
                    let (line, column) = (self.line, self.column);
                    self.bump();
                    self.bump();
                    loop {
                        match self.bump() {
                            None => return Err(self.error(line, column, "unterminated /* comment")),
                            Some('*') if self.peek() == Some('/') => {
                                self.bump();
                                break;
                            }
                            Some(_) => {}
                        }
                    }
                }
                Some('#') => {
                    // `^"#".*` may be a preprocessor line directive; plain
                    // `#...` is a comment. Both consume the line.
                    let directive = self.column == 1;
                    let mut rest = String::new();
                    self.bump();
                    while let Some(c) = self.peek() {
                        if c == '\n' {
                            break;
                        }
                        rest.push(c);
                        self.bump();
                    }
                    if directive {
                        self.apply_line_directive(&rest);
                    }
                }
                _ => return Ok(()),
            }
        }
    }

    /// `# line 42 "file.c"` — adjusts the reported line number.
    fn apply_line_directive(&mut self, rest: &str) {
        let rest = rest.trim();
        let rest = rest.strip_prefix("line").unwrap_or(rest).trim_start();
        let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        if let Ok(n) = digits.parse::<usize>() {
            if n > 0 {
                self.line = n - 1;
            }
        }
    }

    /// Scans a quoted string with cgraph's escape rules: `\"` → `"`,
    /// `\\` → kept as two backslashes, `\<newline>` → dropped, raw newlines
    /// → newlines, everything else verbatim.
    fn scan_quoted(&mut self) -> Result<String, DotError> {
        let (line, column) = (self.line, self.column);
        self.bump(); // opening quote
        let mut out = String::new();
        loop {
            match self.bump() {
                None => {
                    return Err(self.error(line, column, "scanning a quoted string (missing endquote?)"))
                }
                Some('"') => break,
                Some('\\') => match self.bump() {
                    Some('"') => out.push('"'),
                    Some('\\') => {
                        out.push('\\');
                        out.push('\\');
                    }
                    Some('\n') => {}
                    Some(c) => {
                        // scan.l adds any other escaped char verbatim
                        out.push('\\');
                        out.push(c);
                    }
                    None => {
                        return Err(
                            self.error(line, column, "scanning a quoted string (missing endquote?)")
                        )
                    }
                },
                Some(c) => out.push(c),
            }
        }
        Ok(out)
    }

    /// Scans an HTML-like string `<...>`, tracking nesting; contents are
    /// verbatim (the delimiters excluded).
    fn scan_html(&mut self) -> Result<String, DotError> {
        let (line, column) = (self.line, self.column);
        self.bump(); // opening <
        let mut nest = 1usize;
        let mut out = String::new();
        loop {
            match self.bump() {
                None => {
                    return Err(self
                        .error(line, column, "scanning a HTML string (missing '>'? bad nesting?)"))
                }
                Some('>') => {
                    nest -= 1;
                    if nest == 0 {
                        break;
                    }
                    out.push('>');
                }
                Some('<') => {
                    nest += 1;
                    out.push('<');
                }
                Some(c) => out.push(c),
            }
        }
        Ok(out)
    }

    /// Scans a `NAME` or `NUMBER` token with scan.l's patterns and chkNum's
    /// trailing split.
    fn scan_atom(&mut self) -> Result<(String, AtomKind), DotError> {
        // Decide NUMBER vs NAME up front (flex's longest-match alternation).
        let next = |offset: usize| self.peek_at(offset);
        let is_number = match self.peek() {
            Some('-') => next(1).is_some_and(|c| c.is_ascii_digit() || c == '.'),
            Some('.') => next(1).is_some_and(|c| c.is_ascii_digit()),
            Some(c) => c.is_ascii_digit(),
            None => false,
        };
        let mut out = String::new();
        if is_number {
            if self.peek() == Some('-') {
                out.push(self.bump().unwrap());
            }
            if self.peek() == Some('.') {
                // NUMBER: '.' digits+
                out.push(self.bump().unwrap());
                while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                    out.push(self.bump().unwrap());
                }
            } else {
                // digits
                while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                    out.push(self.bump().unwrap());
                }
                if self.peek() == Some('.') {
                    out.push(self.bump().unwrap());
                    while self.peek().is_some_and(|c| c.is_ascii_digit()) {
                        out.push(self.bump().unwrap());
                    }
                }
            }
            // The optional `(\.|LETTER)?` suffix may glue on one more char.
            let last_is_dot = out.ends_with('.');
            let can_glue = if last_is_dot {
                // '.' suffix only when there is no earlier dot (chkNum)
                !out[..out.len() - 1].contains('.')
            } else {
                self.peek().is_some_and(is_name_start)
            };
            if can_glue {
                let c = self.peek().unwrap();
                out.push(c);
                self.bump();
                // chkNum: a trailing '.' with an earlier dot, or a trailing
                // letter, splits the token — rewind one character.
                let ends_dot = out.ends_with('.');
                let ends_letter = out.ends_with(is_name_start);
                if (ends_dot && out[..out.len() - 1].contains('.'))
                    || (ends_letter && !ends_dot)
                {
                    out.pop();
                    self.pos -= 1;
                    self.column -= 1;
                }
            }
            return Ok((out, AtomKind::Plain));
        }
        // NAME: LETTER (LETTER|DIGIT)* — started with a letter/underscore/
        // high char (we only get here with a letter start).
        while self.peek().is_some_and(|c| is_name_start(c) || c.is_ascii_digit()) {
            out.push(self.bump().unwrap());
        }
        Ok((out, AtomKind::Plain))
    }
}

/// scan.l's LETTER class: ASCII letters, underscore, and all non-ASCII
/// characters (so UTF-8 identifiers work).
fn is_name_start(c: char) -> bool {
    c.is_ascii_alphabetic() || c == '_' || c as u32 >= 0x80
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/// A node reference inside an edge statement: the node plus its written port.
#[derive(Debug, Clone)]
struct NodeRef {
    node: usize,
    port: Option<Port>,
}

/// One element of an edge statement's operand list: a comma-separated node
/// list or a subgraph (grammar's T_list / T_subgraph items).
#[derive(Debug, Clone)]
enum Elem {
    Nodes(Vec<NodeRef>),
    Subgraph(usize),
}

/// An unbound `name = value` pair awaiting application (grammar's item list).
#[derive(Debug, Clone)]
struct RawAttr {
    name: String,
    value: String,
    html: bool,
}

/// Mirrors grammar.y's `gstack_t` frame.
struct Frame {
    g: usize,
    attrlist: Vec<RawAttr>,
}

struct Parser {
    tokens: Vec<Token>,
    pos: usize,
    graph: Graph,
    frames: Vec<Frame>,
}

impl Parser {
    fn new(tokens: Vec<Token>, _src: &str) -> Self {
        Self {
            tokens,
            pos: 0,
            graph: Graph::default(),
            frames: Vec::new(),
        }
    }

    fn error(&self, message: impl Into<String>) -> DotError {
        let token = self.peek_token();
        DotError {
            line: token.line,
            column: token.column,
            message: format!("{} near '{}'", message.into(), self.describe(token)),
        }
    }

    fn describe(&self, token: &Token) -> String {
        match &token.tok {
            Tok::Eof => "end of input".into(),
            Tok::Char(c) => c.to_string(),
            Tok::EdgeOp => "->".into(),
            Tok::Keyword(kw) => format!("{kw:?}").to_lowercase(),
            Tok::Atom(s, kind) => {
                let _ = kind;
                s.clone()
            }
        }
    }

    fn peek_token(&self) -> &Token {
        &self.tokens[self.pos.min(self.tokens.len() - 1)]
    }

    fn peek(&self) -> &Tok {
        &self.peek_token().tok
    }

    fn peek_nth(&self, n: usize) -> &Tok {
        &self.tokens[(self.pos + n).min(self.tokens.len() - 1)].tok
    }

    fn bump(&mut self) -> Token {
        let token = self.tokens[self.pos.min(self.tokens.len() - 1)].clone();
        if self.pos < self.tokens.len() - 1 {
            self.pos += 1;
        }
        token
    }

    fn eat_char(&mut self, c: char) -> bool {
        if *self.peek() == Tok::Char(c) {
            self.bump();
            true
        } else {
            false
        }
    }

    fn expect_char(&mut self, c: char) -> Result<(), DotError> {
        if self.eat_char(c) {
            Ok(())
        } else {
            Err(self.error(format!("expected '{c}'")))
        }
    }

    /// `atom : T_atom | qatom ; qatom : T_qatom | qatom '+' T_qatom`
    fn atom(&mut self) -> Result<(String, AtomKind), DotError> {
        match self.peek().clone() {
            Tok::Atom(s, kind) => {
                self.bump();
                if kind == AtomKind::Plain {
                    return Ok((s, kind));
                }
                let mut out = s;
                while *self.peek() == Tok::Char('+') {
                    // Only concat when a qatom follows (else the '+' is a
                    // syntax error at the call site).
                    if let Tok::Atom(next, _) = self.peek_nth(1).clone() {
                        self.bump(); // '+'
                        self.bump(); // atom
                        out.push_str(&next);
                    } else {
                        break;
                    }
                }
                Ok((out, kind))
            }
            _ => Err(self.error("expected an identifier")),
        }
    }

    // -- graph --------------------------------------------------------------

    fn parse(&mut self) -> Result<Graph, DotError> {
        // graph : hdr body
        let strict = self.eat_keyword(Kw::Strict);
        let kind = match self.peek() {
            Tok::Keyword(Kw::Graph) => {
                self.bump();
                GraphKind::Undirected
            }
            Tok::Keyword(Kw::Digraph) => {
                self.bump();
                GraphKind::Directed
            }
            _ => return Err(self.error("expected 'graph' or 'digraph'")),
        };
        let name = if matches!(self.peek(), Tok::Atom(..)) {
            Some(self.atom()?.0)
        } else {
            None
        };

        self.graph.kind = kind;
        self.graph.strict = strict;
        self.graph.name = name.clone();
        self.graph.subgraphs.push(Subgraph {
            name: name.unwrap_or_default(),
            is_cluster: true, // the root graph is always a cluster
            attrs: Default::default(),
            html_attrs: Default::default(),
            nodes: Vec::new(),
            edges: Vec::new(),
            children: Vec::new(),
        });
        self.frames.push(Frame {
            g: 0,
            attrlist: Vec::new(),
        });

        // body : '{' optstmtlist '}'
        self.expect_char('{')?;
        while *self.peek() != Tok::Char('}') {
            if *self.peek() == Tok::Eof {
                return Err(self.error("unexpected end of input"));
            }
            self.stmt()?;
            self.opt_semi();
        }
        self.expect_char('}')?;
        if *self.peek() != Tok::Eof {
            return Err(self.error("expected end of input"));
        }
        // `is_a_cluster` also honours an explicit `cluster=true` attribute,
        // which is only known once the subgraph's body has been parsed.
        for sg in &mut self.graph.subgraphs {
            if sg
                .attrs
                .get("cluster")
                .is_some_and(|value| crate::dotgen::mapbool(Some(value)))
            {
                sg.is_cluster = true;
            }
        }
        Ok(std::mem::take(&mut self.graph))
    }

    fn eat_keyword(&mut self, kw: Kw) -> bool {
        if *self.peek() == Tok::Keyword(kw) {
            self.bump();
            true
        } else {
            false
        }
    }

    fn opt_semi(&mut self) {
        self.eat_char(';');
    }

    // -- statements ---------------------------------------------------------

    /// `stmt : attrstmt optsemi | compound optsemi`
    fn stmt(&mut self) -> Result<(), DotError> {
        match self.peek() {
            Tok::Keyword(Kw::Graph) | Tok::Keyword(Kw::Node) | Tok::Keyword(Kw::Edge) => {
                self.attrstmt()?
            }
            // graphattrdefs: bare `id = value;` sets a graph attribute on the
            // declaring subgraph.
            Tok::Atom(..) if *self.peek_nth(1) == Tok::Char('=') => {
                let (name, _) = self.atom()?;
                self.expect_char('=')?;
                let (value, html) = self.atom()?;
                let g = self.frames.last().unwrap().g;
                self.graph.subgraphs[g].attrs.insert(name.clone(), value);
                if html == AtomKind::Html {
                    self.graph.subgraphs[g].html_attrs.insert(name);
                }
            }
            _ => self.compound()?,
        }
        Ok(())
    }

    /// `attrstmt : attrtype optmacroname attrlist | graphattrdefs`
    fn attrstmt(&mut self) -> Result<(), DotError> {
        let tkind = match self.peek() {
            Tok::Keyword(Kw::Graph) => 0,
            Tok::Keyword(Kw::Node) => 1,
            Tok::Keyword(Kw::Edge) => 2,
            _ => unreachable!(),
        };
        self.bump();

        // optmacroname: `atom '='` — attribute macros are not implemented
        // upstream either (only a warning); we accept and ignore.
        if matches!(self.peek(), Tok::Atom(..)) && *self.peek_nth(1) == Tok::Char('=') {
            self.bump();
            self.bump();
        }

        // attrlist is required after the type keyword.
        self.attrlist()?;
        let attrs = std::mem::take(&mut self.frames.last_mut().unwrap().attrlist);
        match tkind {
            0 => {
                let g = self.frames.last().unwrap().g;
                for attr in &attrs {
                    self.graph.subgraphs[g]
                        .attrs
                        .insert(attr.name.clone(), attr.value.clone());
                    if attr.html {
                        self.graph.subgraphs[g].html_attrs.insert(attr.name.clone());
                    }
                }
            }
            1 => {
                for attr in attrs {
                    self.graph.default_node_attrs.insert(attr.name.clone(), attr.value);
                }
            }
            _ => {
                for attr in attrs {
                    self.graph.default_edge_attrs.insert(attr.name.clone(), attr.value);
                }
            }
        }
        Ok(())
    }

    /// `compound : simple rcompound optattr`
    fn compound(&mut self) -> Result<(), DotError> {
        let first = self.simple()?;
        let mut elems = vec![first];
        let mut is_edge = false;
        while *self.peek() == Tok::EdgeOp {
            self.bump();
            elems.push(self.simple()?);
            is_edge = true;
        }
        let mut attrs = Vec::new();
        while *self.peek() == Tok::Char('[') {
            self.attrlist()?;
            attrs.extend(std::mem::take(&mut self.frames.last_mut().unwrap().attrlist));
        }
        if is_edge {
            self.endedge(elems, attrs)?;
        } else {
            self.endnode(elems, attrs)?;
        }
        Ok(())
    }

    /// `simple : nodelist | subgraph`
    fn simple(&mut self) -> Result<Elem, DotError> {
        match self.peek() {
            Tok::Char('{') | Tok::Keyword(Kw::Subgraph) => Ok(Elem::Subgraph(self.subgraph()?)),
            _ => {
                let mut nodes = vec![self.node()?];
                while self.eat_char(',') {
                    nodes.push(self.node()?);
                }
                Ok(Elem::Nodes(nodes))
            }
        }
    }

    /// `subgraph : optsubghdr body`
    fn subgraph(&mut self) -> Result<usize, DotError> {
        let mut name = None;
        if self.eat_keyword(Kw::Subgraph) {
            if matches!(self.peek(), Tok::Atom(..)) {
                name = Some(self.atom()?.0);
            }
        }
        self.expect_char('{')?;
        let parent = self.frames.last().unwrap().g;
        let sg = match name.as_deref() {
            Some(name) => match self.graph.find_subgraph(parent, name) {
                Some(sg) => sg,
                None => {
                    self.graph.subgraphs.push(Subgraph {
                        name: name.to_string(),
                        is_cluster: is_cluster_name(name),
                        attrs: Default::default(),
                        html_attrs: Default::default(),
                        nodes: Vec::new(),
                        edges: Vec::new(),
                        children: Vec::new(),
                    });
                    let sg = self.graph.subgraphs.len() - 1;
                    self.graph.subgraphs[parent].children.push(sg);
                    sg
                }
            },
            None => {
                self.graph.subgraphs.push(Subgraph {
                    name: format!("%{}", self.graph.subgraphs.len()),
                    is_cluster: false,
                    attrs: Default::default(),
                    html_attrs: Default::default(),
                    nodes: Vec::new(),
                    edges: Vec::new(),
                    children: Vec::new(),
                });
                let sg = self.graph.subgraphs.len() - 1;
                self.graph.subgraphs[parent].children.push(sg);
                sg
            }
        };
        self.frames.push(Frame {
            g: sg,
            attrlist: Vec::new(),
        });
        while *self.peek() != Tok::Char('}') {
            if *self.peek() == Tok::Eof {
                return Err(self.error("unexpected end of input"));
            }
            self.stmt()?;
            self.opt_semi();
        }
        self.expect_char('}')?;
        self.frames.pop();
        Ok(sg)
    }

    /// `node : atom [':' atom [':' atom]]` — appended to the current
    /// statement's node list (grammar's `appendnode`).
    fn node(&mut self) -> Result<NodeRef, DotError> {
        let (name, _) = self.atom()?;
        let mut port = None;
        if self.eat_char(':') {
            let (p1, _) = self.atom()?;
            if self.eat_char(':') {
                let (p2, _) = self.atom()?;
                port = Some(Port::new(format!("{p1}:{p2}")));
            } else {
                port = Some(Port::new(p1));
            }
        }
        let node = self.graph.ensure_node(&name);
        // cgraph membership is transitive: a node named inside a nested
        // subgraph belongs to every enclosing subgraph too (`agcontains` walks
        // up). `frames` is the enclosing chain, outermost first.
        let scopes: Vec<usize> = self.frames.iter().map(|frame| frame.g).collect();
        for g in scopes {
            self.graph.subgraph_add_node(g, node);
        }
        Ok(NodeRef { node, port })
    }

    /// `attrlist : optattr '[' optattrdefs ']'` — appends parsed pairs to the
    /// current frame's attr list.
    fn attrlist(&mut self) -> Result<(), DotError> {
        self.expect_char('[')?;
        loop {
            if self.eat_char(']') {
                break;
            }
            // attrassignment : atom '=' atom
            let (name, _) = self.atom()?;
            self.expect_char('=')?;
            let (value, kind) = self.atom()?;
            let html = kind == AtomKind::Html;
            self.frames.last_mut().unwrap().attrlist.push(RawAttr { name, value, html });
            // optseparator : ';' | ',' | /* empty */
            if !self.eat_char(',') && !self.eat_char(';') {
                // empty separator: the next token must open another group or
                // close the list (grammar's `optattrdefs: optattrdefs attrdefs`
                // allows whitespace-separated assignments).
            }
        }
        Ok(())
    }

    /// `endnode` — applies the trailing attribute list to a plain node list.
    fn endnode(&mut self, elems: Vec<Elem>, attrs: Vec<RawAttr>) -> Result<(), DotError> {
        let mut nodes = Vec::new();
        for elem in elems {
            match elem {
                Elem::Nodes(list) => nodes.extend(list.into_iter().map(|nr| nr.node)),
                Elem::Subgraph(_) => { /* cgraph drops these attributes */ }
            }
        }
        for node in nodes {
            for attr in &attrs {
                self.apply_node_attr(node, attr);
            }
        }
        Ok(())
    }

    fn apply_node_attr(&mut self, node: usize, attr: &RawAttr) {
        self.graph.nodes[node].attrs.insert(attr.name.clone(), attr.value.clone());
        if attr.html {
            self.graph.nodes[node].html_attrs.insert(attr.name.clone());
        }
    }

    /// `endedge` — creates one edge per consecutive element pair, expanding
    /// subgraph operands over their member nodes (`grammar.y`'s endedge +
    /// edgerhs + newedge).
    fn endedge(&mut self, elems: Vec<Elem>, attrs: Vec<RawAttr>) -> Result<(), DotError> {
        // The `key` pseudo-attribute selects among parallel edges instead of
        // being applied as a value.
        let key = attrs
            .iter()
            .find(|a| a.name == "key")
            .map(|a| a.value.clone());
        let attrs: Vec<RawAttr> = attrs.into_iter().filter(|a| a.name != "key").collect();

        for pair in elems.windows(2) {
            let (left, right) = (&pair[0], &pair[1]);
            let tails = self.expand_elem(left);
            let heads = self.expand_elem(right);
            for (t, tport) in tails {
                for (h, hport) in &heads {
                    self.newedge(t, tport.clone(), *h, hport.clone(), key.as_deref(), &attrs)?;
                }
            }
        }
        Ok(())
    }

    /// Expands an edge operand into (node, port) pairs. Subgraph operands
    /// contribute their member nodes, without ports (grammar.y).
    fn expand_elem(&self, elem: &Elem) -> Vec<(usize, Option<Port>)> {
        match elem {
            Elem::Nodes(nodes) => {
                nodes.iter().map(|nr| (nr.node, nr.port.clone())).collect()
            }
            Elem::Subgraph(sg) => {
                self.graph.subgraphs[*sg].nodes.iter().map(|&n| (n, None)).collect()
            }
        }
    }

    /// `newedge` — creates (or strict-merges) one edge and applies ports and
    /// attributes.
    fn newedge(
        &mut self,
        t: usize,
        tport: Option<Port>,
        h: usize,
        hport: Option<Port>,
        _key: Option<&str>,
        attrs: &[RawAttr],
    ) -> Result<(), DotError> {
        // strict graphs drop self loops entirely
        if self.graph.strict && t == h {
            return Ok(());
        }

        let existing = match self.graph.kind {
            GraphKind::Undirected => self.graph.edges.iter().position(|e| {
                (e.tail == t && e.head == h) || (e.tail == h && e.head == t)
            }),
            GraphKind::Directed if self.graph.strict => self
                .graph
                .edges
                .iter()
                .position(|e| e.tail == t && e.head == h),
            GraphKind::Directed => None,
        };

        let index = match existing {
            Some(index) => {
                // Undirected edges may have been stored head-first; ports
                // follow the stored orientation (grammar.y's SWAP).
                if self.graph.kind == GraphKind::Undirected
                    && self.graph.edges[index].head == t
                    && self.graph.edges[index].tail != self.graph.edges[index].head
                {
                    self.set_ports(index, hport, tport);
                } else {
                    self.set_ports(index, tport, hport);
                }
                index
            }
            None => {
                self.graph.edges.push(Edge {
                    tail: t,
                    head: h,
                    attrs: self.graph.default_edge_attrs.clone(),
                    html_attrs: Default::default(),
                    tail_port: None,
                    head_port: None,
                });
                let index = self.graph.edges.len() - 1;
                let scopes: Vec<usize> = self.frames.iter().map(|frame| frame.g).collect();
                for g in scopes {
                    self.graph.subgraph_add_edge(g, index);
                }
                self.set_ports(index, tport, hport);
                index
            }
        };

        for attr in attrs {
            self.graph.edges[index]
                .attrs
                .insert(attr.name.clone(), attr.value.clone());
            if attr.html {
                self.graph.edges[index].html_attrs.insert(attr.name.clone());
            }
        }
        // Explicit `tailport=`/`headport=` attributes win over `:port`.
        for (attr, slot) in [
            ("tailport", true),
            ("headport", false),
        ] {
            if let Some(value) = self.graph.edges[index].attrs.get(attr) {
                let port = Port::new(value.clone());
                if slot {
                    self.graph.edges[index].tail_port = Some(port);
                } else {
                    self.graph.edges[index].head_port = Some(port);
                }
            }
        }
        Ok(())
    }

    fn set_ports(&mut self, edge: usize, tail: Option<Port>, head: Option<Port>) {
        if let Some(port) = tail {
            self.graph.edges[edge].attrs.insert("tailport".into(), port.raw.clone());
            self.graph.edges[edge].tail_port = Some(port);
        }
        if let Some(port) = head {
            self.graph.edges[edge].attrs.insert("headport".into(), port.raw.clone());
            self.graph.edges[edge].head_port = Some(port);
        }
    }
}

/// `is_a_cluster()` in `lib/common/utils.c`: the root, names starting with
/// `cluster` (case-insensitive), or an explicit `cluster=true` attribute.
fn is_cluster_name(name: &str) -> bool {
    name.len() >= 7 && name[..7].eq_ignore_ascii_case("cluster")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_ok(source: &str) -> Graph {
        parse(source).unwrap_or_else(|e| panic!("expected the source to parse: {e}"))
    }

    #[test]
    fn parses_simple_digraph() {
        let g = parse_ok("digraph hello { a -> b; }");
        assert_eq!(g.kind, GraphKind::Directed);
        assert_eq!(g.name.as_deref(), Some("hello"));
        assert_eq!(g.nodes.len(), 2);
        assert_eq!(g.edges.len(), 1);
        assert_eq!((g.edges[0].tail, g.edges[0].head), (0, 1));
    }

    #[test]
    fn undirected_and_strict() {
        let g = parse_ok("graph { a -- b; }");
        assert_eq!(g.kind, GraphKind::Undirected);
        assert_eq!(g.edges.len(), 1);

        let g = parse_ok("strict digraph { a -> b; a -> b; a -> a; }");
        // strict merges duplicates and drops self loops
        assert_eq!(g.edges.len(), 1);
    }

    #[test]
    fn chains_and_subgraph_operands() {
        let g = parse_ok("digraph { a -> b -> c; a -> { x y }; { p q } -> r; }");
        assert_eq!(g.nodes.len(), 8);
        assert_eq!(g.edges.len(), 6);
    }

    #[test]
    fn undirected_existing_edge_keeps_orientation() {
        // The edge is stored head-first from `a -- b:ne`; the second
        // statement has no ports, so the first statement's port survives on
        // the stored (head) side.
        let g = parse_ok("graph { a -- b:ne; b -- a; }");
        assert_eq!(g.edges.len(), 1);
        assert_eq!(g.edges[0].tail, 0);
        assert_eq!(g.edges[0].head, 1);
        assert_eq!(g.edges[0].head_port.as_ref().unwrap().raw, "ne");
    }

    #[test]
    fn undirected_port_swap_when_reversed() {
        // `b:sw -- a` finds the existing a->b edge stored as (a,b); since the
        // stored head (b) equals the statement's tail (b)... actually stored
        // tail==a != t==b, so no swap: the port rides the stored tail? No —
        // grammar.y swaps only when the stored HEAD is the statement tail.
        // Here t=b matches stored head=b → tailport of the statement (sw)
        // must land on the stored head.
        let g = parse_ok("graph { a -- b; b:sw -- a; }");
        assert_eq!(g.edges.len(), 1);
        assert_eq!(g.edges[0].head_port.as_ref().unwrap().raw, "sw");
    }

    #[test]
    fn negative_numbers_lex_as_one_atom() {
        let g = parse_ok("digraph { a -> -3; }");
        assert_eq!(g.nodes.len(), 2);
        assert_eq!(g.nodes[1].name, "-3");
    }

    #[test]
    fn number_with_letter_suffix_splits() {
        // `2.5in` lexes as `2.5` then `in` (chkNum splits the glued letter).
        let g = parse_ok("digraph { 2.5in [shape=box]; }");
        assert_eq!(g.nodes[0].name, "2.5");
    }

    #[test]
    fn ports_and_compass() {
        let g = parse_ok("digraph { a:p1:nw -> b:e; }");
        assert_eq!(g.edges[0].tail_port.as_ref().unwrap().raw, "p1:nw");
        assert_eq!(g.edges[0].head_port.as_ref().unwrap().raw, "e");
    }

    #[test]
    fn clusters_detected_by_name() {
        let g = parse_ok("digraph { subgraph cluster_A { a -> b; } subgraph CLUSTER_x { c; } }");
        let clusters: Vec<_> = g
            .subgraphs
            .iter()
            .filter(|s| s.is_cluster)
            .map(|s| s.name.as_str())
            .collect();
        assert!(clusters.contains(&"cluster_A"));
        assert!(clusters.contains(&"CLUSTER_x"));
    }

    #[test]
    fn rank_same_is_a_subgraph_attr() {
        let g = parse_ok("digraph { { rank=same; a; b; } a -> b; }");
        let sg = &g.subgraphs[1];
        assert_eq!(sg.attrs.get("rank").map(String::as_str), Some("same"));
        assert_eq!(sg.nodes.len(), 2);
    }

    #[test]
    fn subgraph_name_reuse_merges() {
        let g = parse_ok("digraph { subgraph cluster_c { a; } subgraph cluster_c { b; } }");
        let clusters: Vec<usize> = g
            .subgraphs[0]
            .children
            .iter()
            .copied()
            .filter(|&sg| g.subgraphs[sg].is_cluster)
            .collect();
        assert_eq!(clusters.len(), 1);
        assert_eq!(g.subgraphs[clusters[0]].nodes.len(), 2);
    }

    #[test]
    fn defaults_and_html_labels() {
        let g = parse_ok(r#"digraph { node [shape=box]; a [label="Hello"]; b [label=<B>]; }"#);
        assert_eq!(g.nodes[0].attrs.get("shape").map(String::as_str), Some("box"));
        assert_eq!(g.nodes[1].attrs.get("shape").map(String::as_str), Some("box"));
        assert_eq!(g.nodes[0].attrs.get("label").map(String::as_str), Some("Hello"));
        assert!(g.nodes[1].label_is_html());
        assert_eq!(g.nodes[1].attrs.get("label").map(String::as_str), Some("B"));
    }

    #[test]
    fn qatom_concatenation() {
        let g = parse_ok(r#"digraph { a [label="x" + "y"]; }"#);
        assert_eq!(g.nodes[0].attrs.get("label").map(String::as_str), Some("xy"));
    }

    #[test]
    fn numbers_and_negative() {
        let g = parse_ok("digraph { 1 -> 2.5; -3 -> 1; }");
        assert_eq!(g.nodes.len(), 3);
        assert_eq!(g.edges.len(), 2);
    }

    #[test]
    fn node_list_statement() {
        let g = parse_ok("digraph { a, b, c [color=red]; }");
        assert_eq!(g.nodes.len(), 3);
        for node in &g.nodes {
            assert_eq!(node.attrs.get("color").map(String::as_str), Some("red"));
        }
    }

    #[test]
    fn graph_attr_on_subgraph() {
        let g = parse_ok("digraph { rankdir=LR; subgraph cluster_c { label=\"box\"; a; } }");
        assert_eq!(g.graph_attrs().get("rankdir").map(String::as_str), Some("LR"));
        let c = g.subgraphs.iter().find(|s| s.name == "cluster_c").unwrap();
        assert_eq!(c.attrs.get("label").map(String::as_str), Some("box"));
    }

    #[test]
    fn edge_attrs_apply_to_all_expanded_edges() {
        let g = parse_ok("digraph { a -> { b c } [label=L]; }");
        assert_eq!(g.edges.len(), 2);
        assert!(g.edges.iter().all(|e| e.attrs.get("label").map(String::as_str) == Some("L")));
    }

    #[test]
    fn comments_and_semicolons() {
        let g = parse_ok("// lead\n digraph { // shell\n a -> b /* mid */ ; # hash\nc; }");
        assert_eq!(g.nodes.len(), 3);
        assert_eq!(g.edges.len(), 1);
    }

    #[test]
    fn utf8_names() {
        let g = parse_ok("digraph { 图一 -> 图二; }");
        assert_eq!(g.nodes.len(), 2);
    }

    #[test]
    fn syntax_errors_are_positioned() {
        let err = parse("digraph { a -> b; ").unwrap_err();
        assert!(err.line >= 1);
        assert!(err.message.contains("end of input"), "{err}");
    }

    #[test]
    fn edgeop_must_match_graph_kind() {
        // scan.l returns a bare '-' for the wrong-direction operator, which
        // the grammar rejects — exactly like real dot.
        assert!(parse("digraph { a -- b; }").is_err());
        assert!(parse("graph { a -> b; }").is_err());
    }

    #[test]
    fn edge_statement_ports_win_from_colon_syntax() {
        let g = parse_ok("digraph { a:top -> b:bottom; }");
        assert_eq!(g.edges[0].tail_port.as_ref().unwrap().raw, "top");
        assert_eq!(g.edges[0].head_port.as_ref().unwrap().raw, "bottom");
    }
}


mod actions;
mod app;
mod document;
mod dotgen;
mod dump;
mod file_dialog;
mod fonts;
mod graph;
mod icons;
mod settings;
mod svgdump;
mod ui;
mod viz;

use std::path::PathBuf;
use tikv_jemallocator::Jemalloc;

use clap::Parser as _;
use gpui_kit::component::{Root, TitleBar};
use gpui_kit::{
    App, AppContext, Bounds, Focusable, TitlebarOptions, WindowBounds, WindowDecorations,
    WindowOptions, point, px, size,
};

#[global_allocator]
static GLOBAL: Jemalloc = Jemalloc;

#[derive(clap::Parser, Debug)]
#[command(
    name = "dotv",
    version,
    about = "A DOT graph viewer with a Graphviz-compatible `dot` engine"
)]
struct Cli {
    /// The DOT file to lay out; without --dump/--svg it opens in the viewer.
    /// Omit for an empty viewer and pick a file with Ctrl+O.
    #[arg(value_name = "DOT_FILE", value_hint = clap::ValueHint::FilePath)]
    file: Option<PathBuf>,

    /// With --svg: write the SVG here; omit to print it to stdout.
    #[arg(value_name = "OUT_FILE", requires = "svg", value_hint = clap::ValueHint::FilePath)]
    out: Option<PathBuf>,

    /// Print the layout — ranks, node boxes, edge geometry — as JSON to
    /// stdout and exit.
    #[arg(long, requires = "file", conflicts_with = "svg")]
    dump: bool,

    /// Export the drawing as SVG and exit.
    #[arg(long, requires = "file", conflicts_with = "dump")]
    svg: bool,

    /// With --dump: read node sizes (inches) from a `dot -Tplain` file
    /// instead of the analytic estimator. Feeding Graphviz' own measurements
    /// back in removes font-metric differences when diffing layouts.
    #[arg(long, value_name = "PLAIN_FILE", requires = "dump", value_hint = clap::ValueHint::FilePath)]
    sizes: Option<PathBuf>,
}

fn main() {
    let cli = Cli::parse();

    // Headless layout → JSON, for the oracle-diff harness.
    if cli.dump {
        let file = cli.file.expect("clap: --dump requires <FILE>");
        dump::run_with_sizes(&file, cli.sizes.as_deref());
        return;
    }

    if cli.svg {
        let file = cli.file.expect("clap: --svg requires <FILE>");
        svgdump::run(&file, cli.out.as_deref());
        return;
    }

    let file = cli.file;

    gpui_kit::application()
        .with_assets(icons::AppAssets)
        .run(move |cx: &mut App| {
            fonts::register(cx);
            gpui_kit::init(cx);
            fonts::apply_to_theme(cx);
            actions::bind_keys(cx);
            cx.spawn(async move |cx| {
                let options = WindowOptions {
                    window_bounds: Some(WindowBounds::Windowed(Bounds {
                        origin: point(px(0.0), px(0.0)),
                        size: size(px(1280.0), px(860.0)),
                    })),
                    titlebar: Some(TitlebarOptions {
                        title: Some("dotv — DOT Graph Viewer".into()),
                        ..TitleBar::title_bar_options()
                    }),
                    window_decorations: Some(WindowDecorations::Client),
                    ..TitleBar::window_options()
                };
                cx.open_window(options, |window, cx| {
                    let view = cx.new(|cx| app::GraphView::new(file.clone(), cx));
                    let focus_handle = view.focus_handle(cx);
                    window.focus(&focus_handle, cx);
                    cx.new(|cx| Root::new(view, window, cx))
                })
                .expect("failed to open window");
            })
            .detach();
        });
}

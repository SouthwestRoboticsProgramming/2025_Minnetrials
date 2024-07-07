# Robot

## Building Pathfinding

Before simulation will work, Pathfinding needs to be built manually using `cargo build` or `cargo build --release`,
depending on whether `PathfindingJNI.PROFILE` is set to `debug` or `release`. After it has been built once, it does
not need to be rebuilt unless the Rust code is changed.
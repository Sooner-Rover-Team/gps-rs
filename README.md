<!-- cargo-rdme start -->

# `gps-rs`

Bindings to the Swift GPS library ([`gps/`](https://github.com/Sooner-Rover-Team/gps)).

Currently, these are unsafe bindings for use in Rust only. In the future, they should be extended to expose safe bindings with lots of documentation and respect for the (kinda undocumented) safety constraints of the C code. Previous testing shows that violating these unspoken invariants can result in all kinds of weird behavior!

<!-- cargo-rdme end -->

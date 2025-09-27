# Copilot Instructions for obj-renderer-rust

This project is a Rust-based object and material file renderer. To ensure maintainability and clarity, follow these guidelines when contributing code or using AI coding agents:

## Project Structure
- Main entry point: `src/main.rs`
- All source code should reside in the `src/` directory.
- The project is built and run using Cargo (`cargo build`, `cargo run`).

## Documentation
- **Doxygen-style comments are required for all functions, structs, enums, and modules.**
    - Use Rust doc comments (`///`) with Doxygen tags (e.g., `@param`, `@return`, `@brief`).
    - Example:
      ```rust
      /// @brief Renders an object from a file
      /// @param path The path to the object file
      /// @return Result with render status
      fn render_object(path: &str) -> Result<(), RenderError> {
          // ...
      }
      ```
- Update or add comments when modifying code.

## Coding Conventions
- Prefer idiomatic Rust patterns (ownership, error handling with `Result`/`Option`).
- Use clear, descriptive names for functions and variables.
- Group related functionality into modules as the codebase grows.
- Avoid global mutable state.

## Build & Run
- Use standard Cargo commands:
    - Build: `cargo build`
    - Run: `cargo run`
    - Test: `cargo test`
    - Format: `cargo fmt`
    - Lint: `cargo clippy`
- If you add new dependencies, update `Cargo.toml` accordingly.

## Example Patterns
- When adding new features, start with a module and document its public API with Doxygen comments.
- For file parsing, prefer using existing crates (e.g., `obj`, `mtl`) and document integration points.

## External Dependencies
- List all external crates in `Cargo.toml`.
- Document any non-obvious integration steps in the module-level comments.

## Key Files
- `src/main.rs`: Entry point, should be minimal and delegate to well-documented modules.
- `README.md`: Project overview and high-level usage.
- `.github/copilot-instructions.md`: AI agent guidelines (this file).

---

**Always add Doxygen-style comments to new or modified code.**

For questions or unclear conventions, ask for feedback or clarification.

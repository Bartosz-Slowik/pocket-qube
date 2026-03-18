{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";

    rust-overlay.url = "github:oxalica/rust-overlay";
    rust-overlay.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    { self, nixpkgs, ... }@inputs:

    let
      system = "x86_64-linux";
      overlays = [
        (import inputs.rust-overlay)
      ];
      pkgs = import nixpkgs {
        inherit system overlays;
      };

      rust = pkgs.rust-bin.selectLatestNightlyWith (
        toolchain:
        toolchain.default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
          ];
          targets = [
            "thumbv8m.main-none-eabihf"
          ];
        }
      );
    in
    {

      devShells.${system}.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          # language toolchains
          rust
          taplo
          probe-rs-tools

          # debuggers
          gdb
          lldb
        ];
      };

    };
}

with import <nixpkgs> { };

mkShell {
  buildInputs = [
    raylib
    cmake
  ];
}

{
  inputs = {
    nixpkgs.url = "github:vBruegge/nixpkgs/crazyflie";
  };

  outputs = { self, nixpkgs }:
    let
      pkgs = import nixpkgs {
        system = "x86_64-linux";
      };
    in
    {
      devShell.x86_64-linux =
        pkgs.mkShell {
          shellHook = ''
          '';

          buildInputs = with pkgs; [
            gcc-arm-embedded
            openocd
            swig
            ncurses
            pkg-config
            (python310.withPackages (ps: with ps; [
              cflib
              cfclient
            ]))
          ];
        };
    };
}

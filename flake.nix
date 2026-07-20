{
  description = "imu_deadreckoning — Android IMU → UDP → Julia 3-D viewer; strapdown inertial dead reckoning";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs   = import nixpkgs { inherit system; };
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = [
          pkgs.protobuf     # protoc + libprotobuf — the shared message schema
          pkgs.jdk17        # Android Gradle build (Android SDK installed separately)
          pkgs.julia-bin    # SensorLoggerViewer
          # GL runtime for the GLMakie 3-D viewer
          pkgs.libGL
          pkgs.libglvnd
        ];

        # GLMakie needs the GL loader at runtime; expose it without dragging in
        # the system glibc.
        shellHook = ''
          export LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath [
            pkgs.libGL pkgs.libglvnd pkgs.stdenv.cc.cc.lib
          ]}''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          echo "imu_deadreckoning dev shell:"
          echo "  protoc  $(protoc --version)"
          echo "  JAVA_HOME=${pkgs.jdk17}   (Android SDK still required separately)"
          echo "  Julia viewer: tested with juliaup 1.11 (Manifest.toml pins deps);"
          echo "                'julia --project=. gen_proto.jl' then 'viewer3d.jl'."
        '';
      };
    };
}

# Vendored browser libraries

The bench host has no Internet access, so the console serves these files itself.

| File | Source | Version | License |
| --- | --- | --- | --- |
| `three.module.min.js` | npm `three` `build/three.module.min.js` | 0.170.0 | MIT (`LICENSE.three`) |
| `OrbitControls.js`, `STLLoader.js`, `ColladaLoader.js`, `TGALoader.js` | npm `three` `examples/jsm/` | 0.170.0 | MIT (`LICENSE.three`) |
| `URDFLoader.js`, `URDFClasses.js` | npm `urdf-loader` `src/` | 0.13.1 | Apache-2.0 (`LICENSE.urdf-loader`) |

Only change: bare import specifiers (`'three'`, `'three/examples/jsm/...'`,
`'../loaders/TGALoader.js'`) are rewritten to the relative files above, because the
page Content-Security-Policy forbids an inline import map.

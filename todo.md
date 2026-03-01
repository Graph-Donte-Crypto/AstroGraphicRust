- [x] Set planet colors in solar.yml and kerbal.yml instead of in-code const
- [x] `impl From<astrorust_lib::config::Orbit> for Trajectory`
- [x] New methods `Orbit3D::from_state_vectors(mu, r, v) -> Orbit3D` and same for `Trajectory`
- [x] Display time warp using `.draw_text`
- [x] Time warp controls on `[` and `]` keys like in KSP
- [ ] Spacecraft should be able to orbit planets, not just the Sun (to correctly simulate Jupiter gravity assists, as spacecraft spends quite a lot of time in its sphere of influence)
- [ ] Avoid skipping planet SOI on high time warps. 
  - If spacecraft is currently in Sun SOI:
    1. Analytically compute exact SOI entrance and exit times. At each simulation frame:
    2. if `t_SOI_in` < `t` < `t_SOI_out` of some planet, compute spacecraft flyby hyperbola around this planet and compute its position and velocity at time `t` at this hyperbola
- [x] Spacecraft 3D model instead of the damn sphere
- [x] Show spacecraft current orbit using `.draw_text`
- [ ] Update to latest kiss3d
- [ ] Run on wayland
- [ ] Use arcball camera to have KSP experience

# LEAP Hand (vendor copy)

Unmodified copy of `robots/hands/leap_hand/` from https://github.com/dexsuite/dex-urdf
(commit `f5e7132f22108164577fea4c25ef99b5cc0e1900`, 2025-08-17). License: MIT (`LICENSE.txt`).

Copied: `leap_hand_{left,right}.urdf`, `meshes/visual/*.{obj,mtl}`, `meshes/collision/*`.
Not copied: the `*_glb.urdf` variants and `*.glb` meshes.

Why dex-urdf and not the official repo: https://github.com/leap-hand/LEAP_Hand_Sim
(`assets/leap_hand/robot.urdf`) ships the right hand only. The dex-urdf right hand matches it
(same joints, limits and masses, total 0.746 kg; positions rounded to 0.1 mm) and adds a
`base` wrist frame and `*_tip_head` fingertip frames. Its left hand (0.744 kg) has its own
palm mesh, so it is used instead of mirroring the right hand.

Do not edit files here. Put modified meshes (e.g. a reworked palm cover) in a separate
directory and swap them in during generation.

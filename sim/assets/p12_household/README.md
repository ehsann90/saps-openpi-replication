# SIM-P12 household object visuals

These two USD files are visual-only derivatives of existing local LIBERO assets
in the pinned OpenPI subtree. They were converted from OBJ with Isaac Sim 6.1's
asset converter. Their texture references point to the original PNG files in
`third_party/openpi/third_party/libero/`; textures are not duplicated here.
The derived USD stages declare metres as their unit and Z as their up axis,
matching the Isaac scene.

| USD visual | Original mesh | Original texture |
| --- | --- | --- |
| `white_storage_box_visual.usd` | `third_party/openpi/third_party/libero/libero/libero/assets/turbosquid_objects/white_storage_box/white_storage_box.obj` | `ceramic.png` in the same directory |
| `basket_visual.usd` | `third_party/openpi/third_party/libero/libero/libero/assets/stable_scanned_objects/basket/basket.obj` | `texture.png` in the same directory |

The storage box mesh is centered and normalized to a 1 m high visual in its USD;
the scene's 0.04 scale makes it about 39.2 x 33.4 x 40 mm. The basket mesh is
scaled by 0.75 in its USD, giving an outside extent of about
130.1 x 121.6 x 110.7 mm. The scene builder supplies separate primitive
collision geometry: one 40 mm dynamic cuboid for the box and five fixed
cuboids for the basket's floor and walls. The visual meshes have no collision
or rigid-body API.

The source assets are distributed with LIBERO under its MIT license; see
`third_party/openpi/third_party/libero/LICENSE` for the license notice.

## Isaac Sim 6.1 catalog variant

The separate `fr3_droid_robotiq_pick_place_catalog_scene.json` uses these
selected Isaac Sim 6.1 assets. The source URLs are relative to
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/`:

| Role | Catalog source | Local scene asset |
| --- | --- | --- |
| Mac-and-cheese package | `Props/Food/mac_n_cheese.usd` | `mac_n_cheese/visual_upright.usda` |
| Blue plastic basket | `SimReady/Residential/Kitchen/Baskets/Plastic_Basket_A01/sm_misc_basket_plastic_a01_01.usd` | `plastic_basket_a01/visual_only.usd` |
| Mug target | `Props/Mugs/SM_Mug_C1.usd` | `mug_c1/visual_target.usda` |

`mac_n_cheese/mac_n_cheese.usd` is the original catalog USD. Its referenced
`mac_and_cheese.usdz` texture archive came from the Isaac Sim 6.1
`omni.kit.usd.collect` extension test assets because the archive is absent
from the public `Props/Food/` catalog folder. The three original baked PNGs
were extracted from that archive into `mac_n_cheese/textures/`; the derived
`mac_n_cheese_render.usd` changes only their asset paths. The upright wrapper
rotates the source's long Y side onto Z, centers it within a unit collision
cube, and gives a visible size of 37.27 x 16.62 x 73.97 mm when scaled by
the configured collision body.

The basket's `visual_only.usd` is a flattened copy of the selected SimReady
asset's geometry and material stack. It removes the catalog rigid-body and
mesh-collision opinions so that the scene's five fixed primitive colliders
define the open interior. It scales the visual to 138.5 x 176.0 x 74.4 mm.
The three original blue material textures are copied into
`plastic_basket_a01/Textures/`. The catalog basket's CC BY 4.0 license is
preserved in `plastic_basket_a01/LICENSE.txt`.

`mug_c1/SM_Mug_C1.usd` is the original catalog USD with its three referenced
textures copied under `mug_c1/texture/`. Its wrapper applies the selected
0.5 scale. `visual_target.usda` centers the cup around the mug target's
cylindrical rigid body. The package scene leaves `optional_mug.enabled` false;
the separate mug-target scene enables it while disabling `target_object`.
The cup body has a cylindrical collider; the visible handle has no separate
collider.

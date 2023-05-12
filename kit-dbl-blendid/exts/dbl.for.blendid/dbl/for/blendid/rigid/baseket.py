# Fruit baseket
import os
import json
import numpy as np
from pathlib import Path
import omni.kit.app

import omni.usd
from pxr import UsdGeom, Gf, Usd, UsdPhysics
from omni.physx.scripts.utils import setCollider, setRigidBody

EXTENSION_FOLDER_PATH = str(Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
).resolve())

ASSET_FOLDER_PATH = os.path.join(EXTENSION_FOLDER_PATH,"../../../Models/Fruit")

class Basket():
    def __init__(self, item_file_name= "strawberry.usd", generate_path = "/World/Fruit"):
        """
        Basket to generate fruit
        """
        self.item_file_name = item_file_name
        self.generate_path = generate_path

        # stage
        self.stage = omni.usd.get_context().get_stage()

        self.generate_prim = self.stage.GetPrimAtPath(self.generate_path)
        assert self.generate_prim.IsValid(), "Fruit path is not valid"

    def generate_item(self, item_name: str = "strawberry", item_num: int = 1, item_scale: float = 1e-4,
                      random_jitter_scale: float = 0.01, item_z_offset: float = 0.05):
        
        for i in range(item_num):
            item_prim_path = self.generate_path + "/" + item_name + "_" + str(i)
            item_prim = self.stage.GetPrimAtPath(item_prim_path)
            if not item_prim.IsValid():
                item_prim = self.stage.DefinePrim(item_prim_path)

            item_path = os.path.join(ASSET_FOLDER_PATH, self.item_file_name)
            success_bool = item_prim.GetPayloads().AddPayload(item_path)
            assert success_bool, "failed to add payload"

            x, y = random_jitter_scale* np.random.randn(2)
            item_translate = Gf.Vec3d(x, y, item_z_offset + random_jitter_scale * i)
            item_rotation = Gf.Quatd(1, 0, 0, 0)

            item_xform = Gf.Matrix4d().SetScale(Gf.Vec3d(item_scale)) *  \
                Gf.Matrix4d().SetRotate(item_rotation) * Gf.Matrix4d().SetTranslate(item_translate)

            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=item_prim_path,
                new_transform_matrix=item_xform,
            )


            setRigidBody(item_prim, "convexHull", False)
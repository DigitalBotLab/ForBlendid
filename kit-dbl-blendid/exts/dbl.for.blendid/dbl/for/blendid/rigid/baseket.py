# Fruit baseket
import os
import json
import numpy as np
import random
from pathlib import Path
import omni.kit.app

import omni.usd
from pxr import UsdGeom, Gf, Usd, UsdPhysics
from omni.physx.scripts.utils import setCollider, setRigidBody

EXTENSION_FOLDER_PATH = str(Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
).resolve())

ASSET_FOLDER_PATH = os.path.join(EXTENSION_FOLDER_PATH,"../../../Models/Fruit")

from .fruit_config import FRUIT_CONFIG

class Basket():
    def __init__(self, item_file_name= "strawberry", 
                 item_scope = "/World/Fruit", 
                 point_path = "/World/Fruit",
                 item_size = 1e-4,
                 item_index = 0):
        """
        Basket to generate fruit
        """
        # stage
        self.stage = omni.usd.get_context().get_stage()

        self.item_file_name = item_file_name
        self.point_path = point_path
        self.item_scope = item_scope
        self.item_size = item_size if item_size else FRUIT_CONFIG[item_file_name]["item_size"]
        self.item_index = item_index

        # define item scope
        self.item_xform_path = self.item_scope + "/" + self.item_file_name + str( self.item_index)
        self.item_xform = self.stage.GetPrimAtPath(self.item_xform_path)
        if not self.item_xform.IsValid():
            self.item_xform = self.stage.DefinePrim(self.item_xform_path, "Xform")

        self.point_prim = self.stage.GetPrimAtPath(self.point_path)
        assert self.point_prim.IsValid(), "Fruit Point Path is not valid"

        xformable = UsdGeom.Xformable(self.point_prim)
        mat0 = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.point_position = mat0.ExtractTranslation()

    def generate_item(self, item_num: int = 1,
                      random_jitter_scale: float = 0.01, item_z_offset: float = 0.05):
        
        for i in range(item_num):
            item_prim_path =  self.item_xform_path + "/item_" + str(i)
            # print("item_prim_path: ", item_prim_path)
            item_prim = self.stage.GetPrimAtPath(item_prim_path)
            if not item_prim.IsValid():
                item_prim = self.stage.DefinePrim(item_prim_path)

            item_path = os.path.join(ASSET_FOLDER_PATH, self.item_file_name, "model.usd")
            success_bool = item_prim.GetPayloads().AddPayload(item_path)
            assert success_bool, "failed to add payload"

            x, y = random_jitter_scale* np.random.randn(2)
            item_translate = self.point_position + Gf.Vec3d(x, y, item_z_offset + random_jitter_scale * i)
            item_rotation = gen_random_rotation() #Gf.Quatd(1, 0, 0, 0)
            print("item_rotation: ", item_rotation)

            item_xform = Gf.Matrix4d().SetScale(Gf.Vec3d(self.item_size)) *  \
                Gf.Matrix4d().SetRotate(item_rotation) * Gf.Matrix4d().SetTranslate(item_translate)

            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=item_prim_path,
                new_transform_matrix=item_xform,
            )


            setRigidBody(item_prim, "convexHull", False)

    def delete_item(self):
        """
        Delete items from the basket
        """
        omni.kit.commands.execute("DeletePrims", paths=[self.item_xform_path])


def gen_random_rotation():
    """
    Generate random rotation
    """
    # Generate random quaternion rotation
    random_rotation = Gf.Rotation().SetAxisAngle(Gf.Vec3d(0, 0, 1), random.randint(0, 360))
    
    return random_rotation.GetQuat()
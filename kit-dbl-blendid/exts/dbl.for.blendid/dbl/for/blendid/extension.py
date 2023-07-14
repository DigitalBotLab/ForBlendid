import omni.ext
import omni.ui as ui
import omni.timeline
import omni.kit.app
import carb
import numpy as np
import json
import os
import asyncio

# UI
from .ui.style import julia_modeler_style
from .ui.custom_multifield_widget import CustomMultifieldWidget
from .ui.custom_bool_widget import CustomBoolWidget
from .ui.indoorkit_ui_widget import CustomControlGroup
from .rigid.fruit_config import FRUIT_LIST


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class DblForBlendidExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[dbl.for.blendid] dbl for blendid startup")

        # set up fps limit
        carb.settings.get_settings().set_float(
            "/app/runLoops/main/rateLimitFrequency", 25)
        carb.settings.get_settings().set_float(
            "/app/runLoops/present/rateLimitFrequency", 25)
        carb.settings.get_settings().set_bool("/rtx/ecoMode/enabled", True)

        self._window_robot_control = ui.Window("Robot control", width=300)
        self._window_robot_control.frame.style = julia_modeler_style
        with self._window_robot_control.frame:
            with ui.VStack():
                with ui.CollapsableFrame("PLAY", collapsed=True, height=0):
                    with ui.VStack(height=0, spacing=0):
                        ui.Line(style_type_name_override="HeaderLine")
                        ui.Spacer(height=12)
                        control_group = CustomControlGroup()

                        with ui.CollapsableFrame("EE control", collapsed=True):
                            with ui.VStack():
                                self.ee_pos_widget = CustomMultifieldWidget(
                                    label="Transform",
                                    default_vals=[0.4, 0.2, 0.3],
                                    height=20,
                                )
                                ui.Spacer(height=9)
                                with ui.HStack(height=20):
                                    self.ee_ori_widget = CustomMultifieldWidget(
                                        label="Orient (Euler)",
                                        default_vals=[0, 0.0, 0],
                                        height=20,
                                    )
                                ui.Spacer(height=9)
                                ui.Button("Update EE Target", height=20,
                                          clicked_fn=self.update_ee_target)
                                ui.Button("Open/Close Gripper", height=20,
                                          clicked_fn=self.toggle_gripper)

                        with ui.CollapsableFrame("Info", collapsed=True, height=0):
                            with ui.VStack():
                                ui.Line(style_type_name_override="HeaderLine")
                                ui.Spacer(height=12)
                                with ui.HStack(height=20):
                                    self.joint_read_widget = CustomMultifieldWidget(
                                        label="Joint Angle (read only):",
                                        sublabels=["j1", "j2",
                                                   "j3", "j4", "j5", "j6"],
                                        default_vals=[0.0] * 7,
                                        read_only=True
                                    )

                                with ui.HStack(height=20):
                                    self.ee_pos_read_widget = CustomMultifieldWidget(
                                        label="EE Position(read only):",
                                        sublabels=["x", "y", "z"],
                                        default_vals=[0, 0, 0],
                                        read_only=True
                                    )

                                with ui.HStack(height=20):
                                    self.ee_ori_quat_read_widget = CustomMultifieldWidget(
                                        label="EE Quaternion(read only):",
                                        sublabels=["w", "x", "y", "z"],
                                        default_vals=[1, 0, 0, 0],
                                        read_only=True
                                    )
                ui.Button("Record (low level action)", height=50,
                          clicked_fn=self.record_control)
                with ui.HStack(height=20):
                    ui.Label("Action Config Path:", width=200)
                    self.action_config_path_widget = ui.StringField(width=300)
                    self.action_config_path_widget.model.set_value(
                        "task/kinova_action.json")
                with ui.HStack(height=20):
                    ui.Label("Action Name:", width=200)
                    self.action_name_widget = ui.StringField(width=300)
                    self.action_name_widget.model.set_value(
                        "go_home_low_level")

        self._window = ui.Window("For blendid", width=300)
        with self._window.frame:
            with ui.VStack():
                with ui.CollapsableFrame("Motion Play", height=0, collapsed=True):
                    with ui.HStack(height=40):
                        # ui.Button("Motion 1", clicked_fn=self.motion_one)
                        ui.Button("Motion 2", clicked_fn=self.motion_two)
                        ui.Button("Motion 3", clicked_fn=self.motion_three)
                        ui.Button("Motion 4", clicked_fn=self.motion_four)

                with ui.HStack(height=40):
                    ui.Button("Debug", clicked_fn=self.debug)
                    ui.Button("Debug2", clicked_fn=self.debug2)
                with ui.HStack(height=40):
                    ui.Button("Debug Draw Vision", clicked_fn=self.draw_vision)
                    ui.Button("Camera caliberation",
                              clicked_fn=self.camera_caliberation)

                with ui.CollapsableFrame("Fruit Test", height=0, collapsed=True):
                    ui.Line(height=6)
                    with ui.HStack(height=20):
                        ui.Label("Fruit:", width=50)
                        # self.fruit_name_widget = ui.StringField(width = 100)
                        # self.fruit_name_widget.model.set_value("strawberry")

                        self.fruit_name_widget = ui.ComboBox(
                            0, *FRUIT_LIST,
                            name="dropdown_menu",
                            # Abnormal height because this "transparent" combobox
                            # has to fit inside the Rectangle behind it
                            height=10
                        )

                        ui.Label("Size:", width=50)
                        self.fruit_size_widget = ui.FloatField(width=50)
                        self.fruit_size_widget.model.set_value(1e-4)
                        ui.Label("Num:", width=50)
                        self.fruit_num_widget = ui.IntField(width=50)
                        self.fruit_num_widget.model.set_value(10)

                    with ui.HStack(height=40):
                        ui.Button("Add Fruit", clicked_fn=self.fruit_add)
                        ui.Button("Delete Fruit", clicked_fn=self.fruit_delete)
                    ui.Line(height=6)
                    ui.Button("Add Fluid", height=40,
                              clicked_fn=self.fluid_test)

                ui.Line(height=2)
                ui.Button("Register Physics Event", height=50,
                          clicked_fn=self.register_physics_event)
                with ui.HStack(height=20):
                    ui.Label("connect to server:", width=200)
                    self.server_widget = ui.CheckBox(width=20, height=20)

                with ui.HStack(height=20):

                    ui.Label("Robot Prim Path:", width=200)
                    self.robot_path_widget = ui.StringField(width=300)
                    self.robot_path_widget.model.set_value(
                        "/World/kinova_gen3_7_hand/kinova")

                with ui.CollapsableFrame("Robot Config", height=0, collapsed=True):
                    with ui.VStack():
                        with ui.HStack(height=20):
                            ui.Label("EE Prim Path:", width=200)
                            self.ee_path_widget = ui.StringField(width=300)
                            self.ee_path_widget.model.set_value(
                                "/World/kinova_gen3_7_hand/kinova/robotiq_85_base_link")
                        with ui.HStack(height=20):
                            ui.Label("RMP Config Path:", width=200)
                            self.rmp_config_path_widget = ui.StringField(
                                width=300)
                            self.rmp_config_path_widget.model.set_value(
                                "kinova_rmpflow/config7.json")
                        with ui.HStack(height=20):
                            ui.Label("Gripper type:", width=200)
                            self.gripper_type_widget = ui.StringField(
                                width=300)
                            self.gripper_type_widget.model.set_value(
                                "robotiq85")

        # robot
        self.robot = None
        self.controller = None
        self.event_t = 0.0

        # stream
        self._is_stopped = True
        self._tensor_started = False

        # fluid
        self.fluid_instance_idx = 0

    def on_shutdown(self):
        print("[dbl.for.blendid] dbl for blendid shutdown")

    ########################## events #######################################################
    def register_physics_event(self):
        print("register_physics_event")

        # timeline
        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(
            self._on_timeline_event)

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._physics_update_sub = omni.physx.get_physx_interface(
            ).subscribe_physics_step_events(self._on_physics_step)
            self._is_stopped = False

        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_update_sub = None
            self._timeline_sub = None

            self._is_stopped = True
            self._tensor_started = False

            self.robot = None
            self.controller = None

    def _can_callback_physics_step(self) -> bool:
        if self._is_stopped:
            return False

        if self._tensor_started:
            return True

        self._tensor_started = True
        self.set_robot()

        return True

    def _on_physics_step(self, dt):
        self.event_t += dt  # update time

        if not self._can_callback_physics_step():
            return

        if self.controller:
            # print("_on_physics_step")
            self.controller.forward()

            if self.event_t >= 1.0:
                # update joint info
                self.update_robot_ui()
                self.event_t = 0.0

    ########################### robot #######################################################
    def set_robot(self):
        print("set_robot")
        from .ur3e.controller import MyController
        from .ur3e.robot import MyRobot
        from .ur3e.robot_config import ROBOT_CONFIG

        # set robot
        prim_path = self.robot_path_widget.model.as_string
        end_effector_path = self.ee_path_widget.model.as_string
        gripper_type = self.gripper_type_widget.model.as_string
        self.robot = MyRobot(prim_path=prim_path,
                             end_effector_path=end_effector_path,
                             gripper_dof_names=ROBOT_CONFIG[gripper_type]["gripper_dof_names"],
                             gripper_open_position=ROBOT_CONFIG[gripper_type]["gripper_open_position"],
                             gripper_closed_position=ROBOT_CONFIG[gripper_type]["gripper_closed_position"],)
        self.robot.initialize()
        print("robot_info", self.robot.num_dof)
        print("robot_dof_names", len(self.robot.dof_names), self.robot.dof_names)
        # print("robot_gripper", self.robot.gripper._gripper_joint_num)

        # set controller
        rmp_config_path = self.rmp_config_path_widget.model.as_string
        connect_server = self.server_widget.model.as_bool
        self.controller = MyController(name=prim_path,
                                       robot=self.robot,
                                       connect_server=connect_server,
                                       config_path=rmp_config_path)
        # load action config
        if "kinova" in prim_path:
            current_directory = os.path.dirname(os.path.abspath(__file__))
            self.action_config = json.load(
                open(os.path.join(current_directory, "task/kinova_action.json"), "r"))

    def update_ee_target(self):
        print("update_ee_target")
        from .ur3e.numpy_utils import euler_angles_to_quat
        if self.controller:
            self.controller.update_event("move")
            # current_pos, current_rot = self.robot.end_effector.get_world_pose()
            pos = [
                self.ee_pos_widget.multifields[i].model.as_float for i in range(3)]
            rot = [
                self.ee_ori_widget.multifields[i].model.as_float for i in range(3)]

            pos = np.array(pos)  # + np.array(current_pos)
            rot = euler_angles_to_quat(rot, degrees=True)  # xyzw
            # current_rot = np.array([current_rot[1], current_rot[2], current_rot[3], current_rot[0]])
            # rot = quat_mul(current_rot, rot)
            rot = np.array([rot[3], rot[0], rot[1], rot[2]])  # wxyz

            print("updating controller ee target:", pos, rot)
            self.controller.update_ee_target(pos, rot)

    def toggle_gripper(self):
        print("Toggle Gripper")
        if self.controller:
            event = "open" if self.controller.event == "close" else "close"
            self.controller.update_event(event)

    ######################### ui #############################################################
    def update_robot_ui(self):
        """
        read robot joint angles and update ui
        """
        assert self.robot, "robot is not initialized"
        joint_angles = self.robot.get_joint_positions()
        joint_angles = [np.rad2deg(joint_angles[i])
                        for i in range(self.robot.num_dof)]
        self.joint_read_widget.update(joint_angles)
        self.ee_pos_read_widget.update(
            self.robot.end_effector.get_world_pose()[0])
        rot_quat = self.robot.end_effector.get_world_pose()[1]
        self.ee_ori_quat_read_widget.update(rot_quat)
        # rot_euler = quat_to_euler_angles(rot_quat, degrees=True)
        # print("rot_euler:", rot_euler)
        # self.ee_ori_euler_read_widget.update(rot_euler[0])

    def fluid_test(self):
        print(f"[dbl.for.blendid] debug")
        # "/World/Xform"
        from .fluid.faucet import Faucet
        # faucet = Faucet(inflow_path = inflow_path)
        # faucet.set_up_fluid_particle_system()
        # faucet.set_up_cylinder_particles(cylinder_height=1.5, cylinder_radius=0.02)

        faucet = Faucet(material_name="OmniSurface_Honey",
                        inflow_path="/World/blender/juice_point")
        faucet.set_up_fluid_particle_system(
            instance_index=self.fluid_instance_idx)
        faucet.set_up_cylinder_particles(
            cylinder_height=2.0, cylinder_radius=0.02)
        self.fluid_instance_idx += 1

    def fruit_add(self):

        print("fruit_test")
        from .rigid.baseket import Basket
        fruit_index = self.fruit_name_widget.model.get_item_value_model().get_value_as_int()
        fruit_name = FRUIT_LIST[fruit_index]
        fruit_size = self.fruit_size_widget.model.get_value_as_float()
        fruit_num = self.fruit_num_widget.model.get_value_as_int()
        self.basket1 = Basket(item_file_name=fruit_name,
                              point_path="/World/WorkingArea/FruitArea/FruitPoint0",
                              item_size=None)
        print("baseket", self.basket1.point_path)
        self.basket1.generate_item(item_num=fruit_num)

    def fruit_delete(self):
        print("fruit_delete")
        self.basket1.delete_item()

    ########################### motion #######################################################
    def motion_one(self):
        pass

    def motion_two(self):
        if self.controller:
            #  pick_up_blender
            from .ur3e.action_config import action_config

            self.controller.apply_high_level_action(
                action_config["pick_up_blender"])

    def motion_three(self):
        if self.controller:
            #  pick_up_blender
            from .ur3e.action_config import action_config
            place_action = action_config["place_blender_to_point"]
            place_action["base_prim"] = "/World/WorkingArea/FruitArea/FruitPoint0"
            # place_action["base_prim"] = "/World/WorkingArea/LiquidArea/LiquidPoint"
            self.controller.apply_high_level_action(place_action)
            # self.controller.apply_high_level_action("place_blender_to_blending_point")

    def motion_four(self):
        if self.controller:
            #  pick_up_blender
            from .ur3e.action_config import action_config

            pour_action = action_config["pour"]
            pour_action["base_prim"] = "/World/Cup/Xform"
            self.controller.apply_high_level_action(pour_action)

    def record_control(self):
        print("record_control")
        if self.controller:
            self.robot = self.controller.robot
            current_directory = os.path.dirname(os.path.abspath(__file__))
            action_config_path = self.action_config_path_widget.model.as_string
            action_config = json.load(
                open(os.path.join(current_directory, action_config_path), "r"))
            action_name = self.action_name_widget.model.as_string
            new_action = {
                action_name: {
                    "base_prim": None,
                    "steps": [
                        {
                            "action_type": "low_level",
                            "duration": 200,
                            "joint_positions": self.robot.get_joint_positions().tolist()[:(self.robot.num_dof-self.robot.gripper._gripper_joint_num)]
                        }
                    ]
                },
            }
            action_config.update(new_action)
            json.dump(action_config, open(os.path.join(
                current_directory, action_config_path), "w"), indent=4)

    ################################ debug ####################################################

    def debug(self):
        print("debug")
        # if self.controller:
        #     # FIXME: wait for robot to be ready
        #     self.controller.apply_high_level_action(self.action_config["go_home"])

        source_points = [[344, 713], [564, 111], [754, 113]]
        target_points = [[350, 696], [545, 116], [736, 115]]
        from .vision.utils import get_affine_mat
        self.affine_mat = get_affine_mat(source_points, target_points)
        print("affine_mat", self.affine_mat)

    def debug2(self):
        print("debug2")
        if self.controller:
            pick_up_action = self.action_config["pick_up"]
            pick_up_action["base_prim"] = "/World/glass"
            self.controller.apply_high_level_action(pick_up_action)

    def camera_caliberation(self):
        print("camera_caliberation")
        from .vision.vision_helper import VisionHelper
        self.vision_helper = VisionHelper(vision_url="http://127.0.0.1:7860/run/predict",
                                          vision_folder="I:\\Temp",
                                          camera_prim_path="/World/Camera",
                                          vision_model="fastsam")

        camera_pos = self.vision_helper.camera_mat.ExtractTranslation()
        print("camera offset", camera_pos)
        bottom_d = self.vision_helper.get_world_direction_from_camera_point(
            545, -116 + 720) 
        bottom_d = bottom_d.GetNormalized()
        print("bottom_d:", bottom_d)

        self.vision_helper.draw_debug_line(camera_pos, bottom_d, length=10)

    def draw_vision(self):
        # print("draw_vision2")
        vision_folder = "C:\\Research\\Temp"  # "I:\\Temp\\VisionTest"

        from .vision.vision_helper import VisionHelper
        self.vision_helper = VisionHelper(vision_url="http://127.0.0.1:7860/run/predict",
                                          vision_folder=vision_folder,  # "I:\\Temp\\VisionTest",
                                          camera_prim_path="/World/Camera",
                                          vision_model="sam")

        self.vision_helper.get_image_from_webcam()

        import cv2
        import os
        import numpy as np
        from pxr import Gf

        from .vision.utils import find_bottom_point, get_box_transform_from_point2

        img_path = None
        print("os.listdir", os.listdir(vision_folder))
        for item in os.listdir(vision_folder):
            print("item:", item)
            if item.endswith(".png") and item.startswith("test"):
                img_path = os.path.join(vision_folder, item)
                break

        assert img_path, "image not found"
        print("img_path:", img_path)
        image = cv2.imread(img_path)
        cv2.imshow('Original Image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        center_x, center_y = self.vision_helper.get_color_center(
            image,
            lower_color=[150, 50, 50],  # [0, 0, 150], #
            upper_color=[179, 255, 255],  # [180, 20, 200]
            contour_bound=[[375, 300], [275, 720], [950, 720], [775, 300]]
        )

        print("Center Position: ({}, {})".format(center_x, center_y))

        import requests
        import base64
        import json

        # Define the URL of the Gradio server
        url = "http://127.0.0.1:7860/run/predict"
        image_file = img_path

        # Set the request payload
        with open(image_file, "rb") as f:
            encoded_string = base64.b64encode(f.read())

        data_byte = "data:image/png;base64," + encoded_string.decode("utf-8")

        # Send the request to the Gradio server
        response = requests.post(url, json={
            "data": [
                data_byte, center_x, center_y, "cuboid"
            ]
        })

        # Get the response data as a Python object
        response_data = response.json()
        print("response_data", response_data)

        # response_data = self.vision_helper.get_prediction_data("I:\\Temp\\0.jpg", "grey tea tower")
        # print(response_data)

        # response_data =  {'data': ['[[[[736, 113]], [[608, 133]], [[591, 151]], [[590, 373]], [[620, 419]], [[646, 419]], [[741, 392]], [[790, 162]]]]'], 'is_generating': False, 'duration': 11.769976139068604, 'average_duration': 11.769976139068604}
        # import json
        # import numpy as np
        contour = json.loads(response_data["data"][0])

        print("countour", contour)
        points = np.array([p[0] for p in contour])

        bottom_point, bottom_idx = find_bottom_point(points)
        left_point = points[bottom_idx -
                            1 if bottom_idx > 0 else len(points) - 1]
        right_point = points[bottom_idx +
                             1 if bottom_idx < len(points) - 1 else 0]
        print("bottom_point", bottom_point)

        # for i, point in enumerate(points):
        # image = cv2.circle(image, point, radius=10, color=(30 * (i + 1), 30 * (i + 1), 30 * (i + 1)), thickness=-1)

        image = cv2.circle(image, bottom_point, radius=10,
                           color=(255, 0, 255), thickness=-1)
        image = cv2.circle(image, left_point, radius=10,
                           color=(255, 255, 0), thickness=-1)
        image = cv2.circle(image, right_point, radius=10,
                           color=(0, 255, 255), thickness=-1)

        cv2.imshow('Selected Contours', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        def pad(x): return np.hstack([x, np.ones((x.shape[0], 1))])
        def unpad(x): return x[:, :-1]
        def transform(x): return unpad(np.dot(pad(x), self.affine_mat))

        bottom_point_ov = transform(np.array([bottom_point]))[0]
        left_point_ov = transform(np.array([left_point]))[0]
        right_point_ov = transform(np.array([right_point]))[0]

        # bottom_point_ov = self.affine_mat @ np.array([bottom_point[0], bottom_point[1], 1])
        # left_point_ov = self.affine_mat @ np.array([left_point[0], left_point[1], 1])
        # right_point_ov = self.affine_mat @ np.array([right_point[0], right_point[1], 1])

        print("bottom_point_ov", bottom_point_ov, bottom_point)
        print("left_point_ov", left_point_ov, left_point)
        print("right_point_ov", right_point_ov, right_point)

        bottom_point = [bottom_point_ov[0], bottom_point_ov[1]]
        left_point = [left_point_ov[0], left_point_ov[1]]
        right_point = [right_point_ov[0], right_point_ov[1]]

        # REFERENCE: Camera Calibration and 3D Reconstruction from Single Images Using Parallelepipeds

        camera_pos = self.vision_helper.camera_mat.ExtractTranslation()
        print("camera offset", camera_pos)
        bottom_d = self.vision_helper.get_world_direction_from_camera_point(
            bottom_point[0], 720 - bottom_point[1])
        bottom_d = bottom_d.GetNormalized()
        print("bottom_d:", bottom_d)

        left_d = self.vision_helper.get_world_direction_from_camera_point(
            left_point[0], 720 - left_point[1])
        left_d = left_d.GetNormalized()
        print("left_d:", left_d)

        right_d = self.vision_helper.get_world_direction_from_camera_point(
            right_point[0], 720 - right_point[1])
        right_d = right_d.GetNormalized()
        print("right_d:", right_d)

        # self.vision_helper.get_hit_position(t, world_d, target_prim_path="/World/Desk")

        # box_transform, box_rotation = get_box_transform_from_point(camera_pos, bottom_d, left_d, affordance_z = -0.02)

        box_position, box_rotation = get_box_transform_from_point2(
            camera_pos, bottom_d, left_d, right_d, affordance_z=-0.02)
        print("box_transform:", box_position)
        print("box_rotation:", box_rotation)

        # debug draw line
        box_d = box_position - camera_pos
        box_d = box_d.GetNormalized()
        self.vision_helper.draw_debug_line(camera_pos, box_d, length=10)

        mat = Gf.Matrix4d().SetScale(0.01) * \
            Gf.Matrix4d().SetRotate(box_rotation) * \
            Gf.Matrix4d().SetTranslate(
                Gf.Vec3d(box_position[0], box_position[1], box_position[2]))

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path="/World/glass",
            new_transform_matrix=mat,
        )

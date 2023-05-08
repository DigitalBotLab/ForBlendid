import omni.ext
import omni.ui as ui
import omni.timeline
import omni.kit.app
import carb

# UI
from .ui.style import julia_modeler_style
from .ui.custom_multifield_widget import CustomMultifieldWidget
from .ui.custom_bool_widget import CustomBoolWidget

# Robot
from .ur3e.ur3e import UR3E

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class DblForBlendidExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[dbl.for.blendid] dbl for blendid startup")

        # set up fps limit
        carb.settings.get_settings().set_float("/app/runLoops/main/rateLimitFrequency", 30) 
        carb.settings.get_settings().set_float("/app/runLoops/present/rateLimitFrequency", 30) 
        carb.settings.get_settings().set_bool("/rtx/ecoMode/enabled", True)
    
        self._window = ui.Window("For blendid", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Button("Debug", height = 20, clicked_fn=self.debug)

                ui.Line(height = 2)
                ui.Button("Register Physics Event", height = 50, clicked_fn=self.register_physics_event)
                with ui.HStack(height = 20): 
                    ui.Label("Robot Prim Path:", width = 200)
                    self.robot_path_widget = ui.StringField(width = 300)
                    self.robot_path_widget.model.set_value("/World/ur3e")
                
                ui.Spacer(height = 9)
                ui.Label("End Effector", height = 20)
                with ui.HStack(height = 20):
                    self.ee_pos_widget = CustomMultifieldWidget(
                        label="Transform",
                        default_vals=[0, 0, 0],
                        height = 20,
                    )
                ui.Spacer(height = 9)
                with ui.HStack(height = 20):
                    self.ee_ori_widget = CustomMultifieldWidget(
                        label="Orient (Euler)",
                        default_vals=[90, 0.0, 90],
                        height = 20,
                    )
                ui.Spacer(height = 9)
                ui.Button("Update EE Target", height = 20, clicked_fn=self.update_ee_target)
                ui.Button("Open/Close Gripper", height = 20, clicked_fn=self.toggle_gripper)


        # robot
        self.robot = None
        self.controller = None  
        self.event_t = 0.0

        # stream
        self._is_stopped = True
        self._tensor_started = False   

    def on_shutdown(self):
        print("[dbl.for.blendid] dbl for blendid shutdown")

    ########################## events #######################################################
    def register_physics_event(self):
        print("register_physics_event")
        
        # timeline
        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._physics_update_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
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
        self.event_t += dt # update time

        if not self._can_callback_physics_step():
            return

        if self.controller:
            # print("_on_physics_step")
            self.controller.forward()


            if self.event_t >= 1.0:
                # update joint info 
                # self.update_robot_ui()
                self.event_t = 0.0
    
    ########################### robot #######################################################
    def set_robot(self):
        print("set_robot")

        # set robot
        prim_path = self.robot_path_widget.model.as_string
        self.robot = UR3E(prim_path = prim_path)
        self.robot.initialize()
        print("robot_info", self.robot.num_dof)
        print("robot_dof_names", len(self.robot.dof_names), self.robot.dof_names)
        print("robot_gripper", self.robot.gripper._gripper_joint_num)

        # set controller
        # self.controller = CoffeeMakerController("task_controller", self.robot, connect_server=self.server_widget.value)

    ############################################# Robot #######################################
    def update_ee_target(self):
        print("update_ee_target")

    def toggle_gripper(self):
        print("Toggle Gripper")
        if self.controller:
            event = "open" if self.controller.event == "close" else "close"
            self.controller.update_event(event) 
 
    def debug(self):
        print(f"[dbl.for.blendid] debug")
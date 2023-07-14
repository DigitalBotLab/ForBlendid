# send message to Kinova Server to control the real robot
try:
    import cv2
except:
# omni.kit.pipapi extension is required
    import omni.kit.pipapi
    # It wraps `pip install` calls and reroutes package installation into user specified environment folder.
    # That folder is added to sys.path.
    # Note: This call is blocking and slow. It is meant to be used for debugging, development. For final product packages
    # should be installed at build-time and packaged inside extensions.
    omni.kit.pipapi.install(
        package="opencv-python",
    )
    import cv2
 
from PIL import Image
import requests
import base64
import os
import numpy as np

import omni.usd
import carb 
from pxr import Gf, UsdGeom

import omni.timeline
import omni.graph.core as og
from omni.physx import get_physx_scene_query_interface
from omni.debugdraw import get_debug_draw_interface

CX = 1280/2 # principal point x
CY = 720/2 # principal point y
FOC = 1100 # focal length

class VisionHelper():
    def __init__(self, 
                 vision_url: str, 
                 vision_folder:str, 
                 camera_prim_path = "/OmniverseKit_Persp",
                 vision_model = "dino",
                 image_name = "test.png") -> None:
        # vision
        self.vision_url = vision_url
        self.vision_folder = vision_folder
        self.vision_model = vision_model
        self.camera_prim_path = camera_prim_path
        
        # stage
        self.stage = omni.usd.get_context().get_stage()
        self.image_name = image_name

        # get camera transform
        self.obtain_camera_transform()

    def get_prediction_data(self, image_file: str, object_name: str):
        """
        Get bounding box data from the Gradio server
        """

        # Set the request payload
        with open(image_file, "rb") as f:
            encoded_string = base64.b64encode(f.read())

        data_url = "data:image/png;base64," + encoded_string.decode("utf-8")
        payload = {
            "data": [
                data_url, object_name
            ]
        }

        # Send the request to the Gradio server
        response = requests.post(self.vision_url, json=payload)

        # Get the response data as a Python object
        response_data = response.json()

        # Print the response data
        # print(response_data)
        return response_data
    
    def get_image_from_webcam(self):
        """
        Get image from webcam
        """
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame, 'RGB')
        image.save(self.vision_folder + f"/{self.image_name}")
        print("Image saved at path: " + self.vision_folder + f"/{self.image_name}")
        cap.release()
        
    def obtain_camera_transform(self):
        """
        Obtain camera transform
        """
        camera_prim = omni.usd.get_context().get_stage().GetPrimAtPath(self.camera_prim_path)
        xformable = UsdGeom.Xformable(camera_prim)
        self.camera_mat = xformable.ComputeLocalToWorldTransform(0)
        
    def get_world_direction_from_camera_point(self, x, y, fx=FOC, fy=FOC):
        """
        Get world direction from camera point
        """
        # camera_point = Gf.Vec3d(x, y, 1)
        # K = Gf.Matrix3d(fx, 0, 0, 0, fy, 0, CX, CY, 1)
        # K_inverse = K.GetInverse()
        Z = -1
        R = self.camera_mat.ExtractRotationMatrix()
        R_inverse = R.GetInverse()
        # world_point = (camera_point * K_inverse - t) * R_inverse
        D = Gf.Vec3d((CX - x) * Z / fx, (CY - y) * Z / fy, Z)
        world_direction = R_inverse * D 

        return world_direction 
    
    def get_camera_point_from_world_point(self, world_point, fx=FOC, fy=FOC):
        R = self.camera_mat.ExtractRotationMatrix()
        K = Gf.Matrix3d(fx, 0, 0, 0, fy, 0, CX, CY, 1)
        K_inverse = K.GetInverse()

    
    def draw_debug_line(self, origin, direction, length = 1, node_path = "/World/PushGraph/make_array"):
        """
        Draw debug line
        """
        make_array_node = og.Controller.node(node_path)
        if make_array_node.is_valid():
            # print("draw debug line")
            origin_attribute = make_array_node.get_attribute("inputs:input0")
            target_attribute = make_array_node.get_attribute("inputs:input1")
            size_attribute = make_array_node.get_attribute("inputs:arraySize")
            # attr_value = og.Controller.get(attribute)
            og.Controller.set(size_attribute, 2)
            og.Controller.set(origin_attribute, [origin[0], origin[1], origin[2]])
            og.Controller.set(target_attribute, [direction[0] * length + origin[0], direction[1] * length + origin[1], direction[2] * length + origin[2]])
            
            # print("attr:", attr_value)

    def get_hit_position(self, origin, direction, target_prim_path = "/World/Desk"):
        """
        Get hit position
        note: should be call while timeline is playing
        """
        timeline = omni.timeline.get_timeline_interface()
        assert timeline.is_playing(), "timeline is not playing"
        def report_all_hits(hit):
            usdGeom = UsdGeom.Mesh.Get(self.stage, hit.rigid_body)
            print("hit:", hit.rigid_body, usdGeom.GetPrim().GetPath(), hit.position, hit.normal, hit.distance, hit.face_index)
            if usdGeom.GetPrim().GetPath().pathString == target_prim_path:
                hit_position = hit.position

        hit_position = None
        t = carb.Float3(origin[0], origin[1], origin[2])
        d = carb.Float3(direction[0], direction[1], direction[2])
        # print("t:", t, "d:", d)
        get_physx_scene_query_interface().raycast_all(t, d, 100.0, report_all_hits)

        return hit_position
    
    def get_color_center(self, 
                    image,
                    lower_color, 
                    upper_color,
                    contour_bound = [[0, 0],[0, 720], [1280, 720], [1280, 0]]):

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        cv2.imshow('Original hsv_image Image', hsv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        lower_color = np.array(lower_color)   
        upper_color = np.array(upper_color)
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  
        
        filtered_image = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow('Selected color mask', filtered_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        assert len(contours) > 0, "contour not found"

        cv2.drawContours(filtered_image, contours, -1, (255, 255, 255), 2)
        cv2.imshow('Contours', filtered_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        contour_bound = np.array(contour_bound)
        largest_area = 0
        x, y = None, None
        for contour in contours:
            # largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(contour)
            
            M = cv2.moments(contour)
            center_x = int(M['m10'] / M['m00'])
            center_y = int(M['m01'] / M['m00'])
            distance = cv2.pointPolygonTest(contour_bound, (center_x, center_y), False)
            # print("contour_area:", contour_area, center_x, center_y, distance)
            if distance > 0:
                if contour_area > largest_area:
                    x, y = center_x, center_y
                    largest_area = contour_area

        
        return x, y
        
    ############################################# action #############################################
    def capture_image(self, folder_path = "I:\\Temp\\VisionTest", image_name = "test"):

        from omni.kit.capture.viewport import CaptureOptions, CaptureExtension

        options = CaptureOptions()
        options.file_name = image_name
        options.file_type = ".png"
        options.output_folder = str(folder_path)

        options.camera = self.camera_prim_path
        
        if not os.path.exists(options.output_folder):
            pass
        images = os.listdir(options.output_folder)
        for item in images:
            if item.endswith(options.file_type) and item.startswith(options.file_name):
                os.remove(os.path.join(options.output_folder, item))

        capture_instance = CaptureExtension().get_instance()
        capture_instance.options = options
        capture_instance.start() 
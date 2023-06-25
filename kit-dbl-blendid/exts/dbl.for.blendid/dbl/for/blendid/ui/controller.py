# controller
import carb

class UIController():
    w = False
    s = False
    a = False
    d = False
    q = False
    e = False

    up = False
    down = False
    left = False
    right = False

    # Controller.scale = 0.1
    left_control = False

    def __init__(self) -> None:
        self.user_control = 0.25 
        self.network_control = 0.25

        UIController.reset_movement()
    
    @classmethod
    def reset_movement(cls):
        UIController.w = False
        UIController.s = False
        UIController.a = False
        UIController.d = False
        UIController.q = False
        UIController.e = False

        UIController.up = False
        UIController.down = False
        UIController.left = False
        UIController.right = False

        # Controller.left_control = False
        
        
    def handle_keyboard_event(self, event):
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            or event.type == carb.input.KeyboardEventType.KEY_REPEAT
            ): 
            # print("event input", event.input)
            if event.input == carb.input.KeyboardInput.W:
                UIController.w = True
            if event.input == carb.input.KeyboardInput.S:
                UIController.s = True
            if event.input == carb.input.KeyboardInput.A:
                UIController.a = True
            if event.input == carb.input.KeyboardInput.D:
                UIController.d = True
            if event.input == carb.input.KeyboardInput.Q:
                UIController.q = True
            if event.input == carb.input.KeyboardInput.E:
                UIController.e = True
                

            if event.input == carb.input.KeyboardInput.UP:
                UIController.up = True
            if event.input == carb.input.KeyboardInput.DOWN:
                UIController.down = True
            if event.input == carb.input.KeyboardInput.LEFT:
                UIController.left = True
            if event.input == carb.input.KeyboardInput.RIGHT:
                UIController.right = True

            if event.input == carb.input.KeyboardInput.LEFT_CONTROL:
                UIController.left_control = True

        if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # print("event release", event.input)
            if event.input == carb.input.KeyboardInput.W:
                UIController.w = False
            if event.input == carb.input.KeyboardInput.S:
                UIController.s = False
            if event.input == carb.input.KeyboardInput.A:
                UIController.a = False
            if event.input == carb.input.KeyboardInput.D:
                UIController.d = False
            if event.input == carb.input.KeyboardInput.Q:
                UIController.q = False
            if event.input == carb.input.KeyboardInput.E:
                UIController.e = False

            if event.input == carb.input.KeyboardInput.UP:
                UIController.up = False
            if event.input == carb.input.KeyboardInput.DOWN:
                UIController.down = False
            if event.input == carb.input.KeyboardInput.LEFT:
                UIController.left = False
            if event.input == carb.input.KeyboardInput.RIGHT:
                UIController.right = False

            if event.input == carb.input.KeyboardInput.LEFT_CONTROL:
                UIController.left_control = False

    def PoolUserControl(self):
        return self.user_control

    def PoolNetworkControl(self):
        return 0.1 if UIController.w else 0.25

    def QueryMove(self):
        move = [0, 0, 0]
        if UIController.w:
            move[0] += 1 
        if UIController.s:
            move[0] -= 1
        if UIController.a:
            move[1] += 1
        if UIController.d:
            move[1] -= 1
        if UIController.q:
            move[2] -= 1
        if UIController.e:
            move[2] += 1

        return move

    def QueryRotation(self):
        rotation = [0, 0]
        if UIController.up:
            rotation[0] += 1 
        if UIController.down:
            rotation[0] -= 1
        if UIController.left:
            rotation[1] += 1
        if UIController.right:
            rotation[1] -= 1

        return rotation
        
    def QueryGripper(self):
        if not UIController.left_control:
            return 1 # open
        else:
            return -1 # close
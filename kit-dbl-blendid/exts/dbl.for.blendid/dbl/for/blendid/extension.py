import omni.ext
import omni.ui as ui
import carb

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
                ui.Button("Add", height = 20, clicked_fn=self.debug)

    def on_shutdown(self):
        print("[dbl.for.blendid] dbl for blendid shutdown")

    def debug(self):
        print(f"[dbl.for.blendid] debug")
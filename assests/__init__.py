# realman_sim/assets/__init__.py
from pathlib import Path

ASSETS_ROOT = Path(__file__).resolve().parent  # .../realman_sim/assets
XMATE_XML= str(ASSETS_ROOT / "xmate_pro7_native.xml")
SCENE_XML = str(ASSETS_ROOT / "scene.xml")
XMATA_URDF = str(ASSETS_ROOT / "xmate_pro7.urdf")


def get_model_path_xml():
    return XMATE_XML

def get_scene_path_xml():
    return SCENE_XML

def get_urdf_path():
    return XMATA_URDF
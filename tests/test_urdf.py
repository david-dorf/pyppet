from pyppet.examples.example_model import EXAMPLE_MODEL
from pyppet.urdf import pyppet_to_urdf
import xml.etree.ElementTree as ET
from yourdfpy import urdf
import io


urdf_str = pyppet_to_urdf(EXAMPLE_MODEL, "example.urdf")
root = ET.fromstring(urdf_str)

def test_urdf_model():
    with io.StringIO(urdf_str) as f:
        urdf_model = urdf.URDF.load(f)  # You can visualize the model with urdf_model.show()
        urdf_model.show()
        assert urdf_model is not None

def test_robot_tag():
    assert root.tag == 'robot'

def test_robot_name():
    assert root.attrib['name'] == 'example_model'

def test_link_name():
    link = root.find('link')
    assert link is not None
    assert link.attrib['name'] == 'base_link'

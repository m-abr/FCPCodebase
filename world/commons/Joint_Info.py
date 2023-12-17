import numpy as np

class Joint_Info():
    def __init__(self, xml_element) -> None:
        self.perceptor = xml_element.attrib['perceptor']
        self.effector =  xml_element.attrib['effector']
        self.axes =      np.array([
                            float(xml_element.attrib['xaxis']), 
                            float(xml_element.attrib['yaxis']), 
                            float(xml_element.attrib['zaxis'])])
        self.min =       int(xml_element.attrib['min'])
        self.max =       int(xml_element.attrib['max'])

        self.anchor0_part = xml_element[0].attrib['part']
        self.anchor0_axes = np.array([
                        float(xml_element[0].attrib['y']), 
                        float(xml_element[0].attrib['x']), 
                        float(xml_element[0].attrib['z'])]) #x and y axes are switched

        self.anchor1_part = xml_element[1].attrib['part']
        self.anchor1_axes_neg = np.array([
                        -float(xml_element[1].attrib['y']), 
                        -float(xml_element[1].attrib['x']), 
                        -float(xml_element[1].attrib['z'])]) #x and y axes are switched

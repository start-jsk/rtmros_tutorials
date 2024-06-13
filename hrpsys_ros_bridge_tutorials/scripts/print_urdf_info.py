import xml.etree.ElementTree as ET
import sys
import math

def parse_urdf(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    links = []
    joints = []
    
    for link in root.findall('link'):
        name = link.get('name')
        
        mass_element = link.find('inertial/mass')
        if mass_element is not None:
            mass = float(mass_element.get('value'))
        else:
            mass = 0.0
        
        inertia_element = link.find('inertial/inertia')
        if inertia_element is not None:
            ixx = float(inertia_element.get('ixx'))
            ixy = float(inertia_element.get('ixy'))
            ixz = float(inertia_element.get('ixz'))
            iyy = float(inertia_element.get('iyy'))
            iyz = float(inertia_element.get('iyz'))
            izz = float(inertia_element.get('izz'))
            inertia = (ixx, ixy, ixz, iyy, iyz, izz)
        else:
            inertia = None
        
        links.append((name, mass, inertia))
    
    for joint in root.findall('joint'):
        name = joint.get('name')
        limit_element = joint.find('limit')
        if limit_element is not None:
            lower = float(limit_element.get('lower', 0.0))
            upper = float(limit_element.get('upper', 0.0))
            effort = float(limit_element.get('effort', 0.0))
            velocity = float(limit_element.get('velocity', 0.0))
            limits = (lower, upper, effort, velocity)
        else:
            limits = (0.0, 0.0, 0.0, 0.0)
        
        joints.append((name, limits))
    
    return links, joints

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py path_to_your_urdf_file.urdf")
        sys.exit(1)
    
    file_path = sys.argv[1]
    links, joints = parse_urdf(file_path)
    
    total_mass = sum(link[1] for link in links)
    
    print("-" * 120)
    print("file_path : {}".format(file_path))
    print("=" * 120)
    print("{:<20} {:<10} {:<70}".format("Link Name", "Mass", "Inertia"))
    print("-" * 120)
    
    for link in links:
        name = link[0]
        mass = link[1]
        if link[2] is not None:
            inertia = "ixx={:+.5f}, ixy={:+.5f}, ixz={:+.5f}, iyy={:+.5f}, iyz={:+.5f}, izz={:+.5f}".format(
                link[2][0], link[2][1], link[2][2], link[2][3], link[2][4], link[2][5])
        else:
            inertia = "None"
        print("{:<20} {:<10.3f} {:<70}".format(name, mass, inertia))
    
    print("-" * 120)
    print("{:<20} {:<10.2f}".format("Total Mass", total_mass))
    print("=" * 120)
    print("\n{:<20} {:<10} {:<10} {:<10} {:<10} {:<10} {:<10}".format("Joint Name", "Lower[rad]", "Upper[rad]", "Lower[deg]", "Upper[deg]", "Effort", "Velocity"))
    print("-" * 120)
    
    for joint in joints:
        name = joint[0]
        limits = joint[1]
        print("{:<20} {:<10.2f} {:<10.2f} {:<10.2f} {:<10.2f} {:<10.2f} {:<10.2f}".format(name, limits[0], limits[1], limits[0]*180/math.pi, limits[1]*180/math.pi, limits[2], limits[3]))

if __name__ == "__main__":
    main()

'''
Module for simulating a door with mujoco
Can also modify the parameters of the door
'''
import argparse
import math
import mujoco_py
import sys
from time import sleep
import xml
import xmltodict


class Point:
    '''For making the scaling stuff easier'''
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def scaled_str(self, scale):
        '''Multiply by 1000 because of units'''
        return '{:.3f} {:.3f} {:.3f}'.format(
            self.x * scale * 1000,
            self.y * scale * 1000,
            self.z * scale * 1000)

# Use the defaults for calculating positions with new scales
DEFAULTS = {
    'scale'             : 0.001,
    'mass'              : 100,
    'stiffness'         : 0.001,
    'joint_door_base'   : Point(0.4, 0.25, 0.052),
    'joint_door_hinge'  : Point(0.005, 0.005, 0.005),
    'joint_handle_core' : Point(0.052, -0.06, 0.095),
    'joint_handle_beam' : Point(0.015, 0.0, -0.0165),
    'joint_handle_latch': Point(-0.015, 0.04, -0.01),
    'joint_frame_left'  : Point(-0.025, 0.01, 0.0),
    'joint_frame_left_f': Point(-0.025, 0.0, 0.0),
    'joint_frame_right' : Point(0.267, 0.0, 0.0),
    'joint_frame_bottom': Point(-0.025, 0.0, -0.052),
    'joint_frame_top'   : Point(-0.025, 0.0, 0.335),
    'pos_body'          : Point(0.48, 0.17, 0.052),
    'pos_door'          : Point(0.005, 0.005, 0.005),
    'pos_door_hinge'    : Point(0.262, 0.049, 0),
    'pos_handle_core'   : Point(0.052, -0.06, 0.095),
    'pos_handle_beam'   : Point(0.015, 0.0, -0.0165),
    'pos_handle_latch'  : Point(-0.015, 0.04, -0.01),
    'pos_frame_left'    : Point(-0.025, 0.01, 0.0),
    'pos_frame_left_f'  : Point(-0.025, 0.0, 0.0),
    'pos_frame_right'   : Point(0.267, 0.0, 0.0),
    'pos_frame_bottom'  : Point(-0.025, 0.0, -0.052),
    'pos_frame_top'     : Point(-0.025, 0.0, 0.335),
    'pos_door_inertia'  : Point(0.130, 0.027, 0.165),
    'pos_frame_inertia' : Point(-0.012, -0.004, -0.052),
    'pos_handle_beam_i' : Point(0, 0.0165, 0),
    'pos_handle_latch_i': Point(0, 0, 0.01)
}


def create_argparse():
    '''Creates an argument parser'''
    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(
        dest='action'
    )

    run_parser = subparsers.add_parser(
        'run',
        help='Run the simulation.'
    )

    run_parser.add_argument(
        '--time',
        help='Number of frames to simulate.',
        default=10000,
        type=int
    )

    run_parser.add_argument(
        '--xml',
        help='Load the model from another xml file.',
        default='robots/samplefinal.xml',
        type=str
    )

    run_parser.add_argument(
        '--info',
        help='Print simulation info to console.',
        nargs='?',
        const=True,
        default=False,
        type=bool
    )

    modify_parser = subparsers.add_parser(
        'modify',
        help='Modify the parameters of the door.'
    )

    modify_parser.add_argument(
        '--default',
        help='Reset the door to default settings.',
        action='store_true'
    )

    modify_parser.add_argument(
        '--mass',
        help='Set the mass of the door.',
        type=int
    )

    modify_parser.add_argument(
        '--stiffness',
        help='Set the stiffness of the handle.',
        type=int
    )

    modify_parser.add_argument(
        '--scale',
        help='Set the scale of the door.',
        default=0.001,
        type=float
    )

    modify_parser.add_argument(
        '--xml',
        help='Set the XML file to modify.',
        default='robots/samplefinal.xml',
        type=str
    )

    modify_parser.add_argument(
        '--xacro',
        help='Set the xacro file to modify.',
        default='robots/robot_door.urdf.xacro',
        type=str
    )

    modify_parser.add_argument(
        '--xml_dest',
        help='File to save the modified XML in.',
        default='robots/samplefinal.xml',
        type=str
    )

    modify_parser.add_argument(
        '--xacro_dest',
        help='File to save the modified xacro in.',
        default='robots/robot_door.urdf.xacro',
        type=str
    )

    return parser


def remove_invalid_args(args):
    '''Remove args with negative or missing values'''
    return {k: v for k, v in args.items() if v is not None and (isinstance(v, str) or v >= 0)}


def simulate(args):
    '''Start the mujoco simulation'''
    model = mujoco_py.load_model_from_path(args['xml'])
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)
    t = 0
    while True:
        t += 1
        sim.step()
        viewer.render()
        if args['info']:
            print('Step {}:'.format(t))
            print(sim.data.ctrl)
        if t > args['time']:
            break


def modify_door(args):
    xml_src = args['xml']
    xacro_src = args['xacro']
    xml_dest = args['xml_dest']
    xacro_dest = args['xacro_dest']
    for key, value in args.items():
        if key == 'mass':
            set_mass(value, xml_src, xml_dest)
        if key == 'stiffness':
            set_stiffness(value, xml_src, xml_dest)
        if key == 'scale':
            set_scales(value, xml_src, xacro_src, xml_dest, xacro_dest)
        if key == 'default':
            set_defaults(xml_src, xacro_src, xml_dest, xacro_dest)


def set_mass(value, src, dest):
    door = load_xml(src)
    for part in door['mujoco']['worldbody']['body']['body']:
        if part['@name'].startswith('frame_'):
            part['inertial']['@mass'] = value
    save_xml(dest, door)


def set_stiffness(value, src, dest):
    door = load_xml(src)
    for part in door['mujoco']['worldbody']['body']['body']:
        if part['@name'] == 'door':
            part['body']['joint']['@stiffness'] = value
    save_xml(dest, door)


def set_scales(value, xml_src, xacro_src, xml_dest, xacro_dest):
    door = load_xml(xml_src)
    xacro = load_xml(xacro_src)
    scale_str = '{} {} {}'.format(value, value, value)
    for part in door['mujoco']['asset']['mesh']:
        part['@scale'] = scale_str

    for part in door['mujoco']['worldbody']['body']['body']:
        if part['@name'] == 'door':
            part['@pos'] = DEFAULTS['pos_door'].scaled_str(value)
            part['inertial']['@pos'] = DEFAULTS['pos_door_inertia'].scaled_str(value)
            part['joint']['@pos'] = DEFAULTS['pos_door_hinge'].scaled_str(value)
            part['body']['@pos'] = DEFAULTS['pos_handle_core'].scaled_str(value)
            for subpart in part['body']['body']:
                if subpart['@name'] == 'handle_beam':
                    subpart['@pos'] = DEFAULTS['pos_handle_beam'].scaled_str(value)
                    subpart['inertial']['@pos'] = DEFAULTS['pos_handle_beam_i'].scaled_str(value)
                elif subpart['@name'] == 'handle_latch':
                    subpart['@pos'] = DEFAULTS['pos_handle_latch'].scaled_str(value)
                    subpart['inertial']['@pos'] = DEFAULTS['pos_handle_latch_i'].scaled_str(value)
        elif part['@name'] == 'frame_left':
            part['@pos'] = DEFAULTS['pos_frame_left'].scaled_str(value)
            part['inertial']['@pos'] = DEFAULTS['pos_frame_inertia'].scaled_str(value)
        elif part['@name'] == 'frame_left_front':
            part['@pos'] = DEFAULTS['pos_frame_left_f'].scaled_str(value)
            part['inertial']['@pos'] = DEFAULTS['pos_frame_inertia'].scaled_str(value)
        elif part['@name'] == 'frame_right':
            part['@pos'] = DEFAULTS['pos_frame_right'].scaled_str(value)
            part['inertial']['@pos'] = DEFAULTS['pos_frame_inertia'].scaled_str(value)
        elif part['@name'] == 'frame_top':
            part['@pos'] = DEFAULTS['pos_frame_top'].scaled_str(value)
            part['inertial']['@pos'] = DEFAULTS['pos_frame_inertia'].scaled_str(value)
        elif part['@name'] == 'frame_bottom':
            part['@pos'] = DEFAULTS['pos_frame_bottom'].scaled_str(value)
            part['inertial']['@pos'] = DEFAULTS['pos_frame_inertia'].scaled_str(value)

    for part in xacro['robot']['link']:
        if part['@name'] in ['base_link', 'door_base']:
            continue
        part['visual']['geometry']['mesh']['@scale'] = scale_str
        part['collision']['geometry']['mesh']['@scale'] = scale_str
    
    for part in xacro['robot']['joint']:
        if part['@name'] == 'door_hinge':
            part['origin']['@xyz'] = DEFAULTS['joint_door_hinge'].scaled_str(value)
        elif part['@name'] == 'handle_core':
            part['origin']['@xyz'] = DEFAULTS['joint_handle_core'].scaled_str(value)
        elif part['@name'] == 'handle_beam_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_handle_beam'].scaled_str(value)
        elif part['@name'] == 'handle_latch_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_handle_latch'].scaled_str(value)
        elif part['@name'] == 'frame_left_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_frame_left'].scaled_str(value)
        elif part['@name'] == 'frame_left_front_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_frame_left_f'].scaled_str(value)
        elif part['@name'] == 'frame_right_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_frame_right'].scaled_str(value)
        elif part['@name'] == 'frame_bottom_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_frame_bottom'].scaled_str(value)
        elif part['@name'] == 'frame_top_joint':
            part['origin']['@xyz'] = DEFAULTS['joint_frame_top'].scaled_str(value)

    save_xml(xml_dest, door)
    save_xml(xacro_dest, xacro)


def set_defaults(xml_src, xacro_src, xml_dest, xacro_dest):
    set_mass(DEFAULTS['mass'], xml_src, xml_dest)
    set_stiffness(DEFAULTS['stiffness'], xml_src, xml_dest)
    set_scales(DEFAULTS['scale'], xml_src, xacro_src, xml_dest, xacro_dest)


def load_xml(filename):
    with open(filename) as xml_file:
        return xmltodict.parse(xml_file.read())


def save_xml(filename, values):
    with open(filename, 'w') as xml_file:
        xml_file.write(xmltodict.unparse(values, pretty=True))


def is_default_arg(key, value):
    return value == create_argparse()._subparsers._group_actions[0].choices['modify'].get_default(key)


def set_dest_file(args):
    if not is_default_arg('xml', args['xml']) and is_default_arg('xml_dest', args['xml_dest']):
        args['xml_dest'] = args['xml']
    if not is_default_arg('xacro', args['xacro']) and is_default_arg('xacro_dest', args['xacro_dest']):
        args['xacro_dest'] = args['xacro']


if __name__ == '__main__':
    parser = create_argparse()
    args = vars(parser.parse_args())
    if len(sys.argv) > 1:
        args = remove_invalid_args(args)
        if args['action'] == 'run':
            simulate(args)
        elif args['action'] == 'modify':
            set_dest_file(args)
            modify_door(args)

    else:
        parser.print_help()

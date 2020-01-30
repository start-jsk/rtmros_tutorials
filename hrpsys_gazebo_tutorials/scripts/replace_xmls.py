#!/usr/bin/env python

import sys, os
from xml.dom.minidom import parse, parseString
import xml.dom
import yaml
import argparse

reload(sys)
sys.setdefaultencoding('utf-8')

def getParentNode(node, depth):
    if depth == 0:
        return node
    else:
        return getParentNode(node.parentNode, depth - 1) 

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='replace_xmls')
    parser.add_argument('filename', nargs=1)
    parser.add_argument('-O', '--output', help='output filename')
    parser.add_argument('-C', '--config', help='config filename (yaml file)')

    args = parser.parse_args()
    obj = xml.dom.minidom.parse(args.filename[0])

    if obj:
        if args.config:
            yaml_data = yaml.load(open(args.config).read())
            if 'replace_xmls' in yaml_data:
                for replace_xml in yaml_data['replace_xmls']:
                    match_rule = replace_xml['match_rule']
                    target_tags = []
                    if match_rule.has_key('tag'):
                        tags = obj.getElementsByTagName(match_rule['tag'])
                        if match_rule.has_key('attribute'):
                            attributes = match_rule['attribute']
                            for attribute in attributes:
                                attribute_name = str(attribute['name'])
                                if attribute.has_key('value'):
                                    attribute_value = str(attribute['value'])
                                    tags = [tag for tag in tags if tag.getAttribute(attribute_name) == attribute_value]
                                else:
                                    for tag in tags:
                                        sys.stderr.write('%s\n' % tag.hasAttribute(attribute_name))
                                    tags = [tag for tag in tags if tag.hasAttribute(attribute_name)]
                                
                        if match_rule.has_key('sub_tag'):
                            if match_rule.has_key('sub_attribute'):
                                sub_attributes = match_rule['sub_attribute']
                                for sub_attribute in sub_attributes:
                                    sub_attribute_name = str(sub_attribute['name'])
                                    if sub_attribute.has_key('value'):
                                        sub_attribute_value = str(sub_attribute['value'])
                                        matched_tags = []
                                        for tag in tags:
                                            for sub_tag in tag.getElementsByTagName(match_rule['sub_tag']):
                                                if sub_tag.getAttribute(sub_attribute_name) == sub_attribute_value:
                                                    matched_tags.append(tag)
                                                    break
                                        tags = matched_tags
                                    else:
                                        matched_tags = []
                                        for tag in tags:
                                            for sub_tag in tag.getElementsByTagName(match_rule['sub_tag']):
                                                if sub_tag.hasAttribute(sub_attribute_name):
                                                    matched_tags.append(tag)
                                                    break
                                        tags = matched_tags
                            else:
                                tags = [tag for tag in tags if len(tag.getElementsByTagName(match_rule['sub_tag'])) > 0]                 

                        if match_rule.has_key('parent_tag'):
                            if match_rule.has_key('parent_attribute'):
                                parent_attributes = match_rule['parent_attribute']
                                for parent_attribute in parent_attributes:
                                    parent_attribute_name = str(parent_attribute['name'])
                                    if parent_attribute.has_key('value'):
                                        parent_attribute_value = str(parent_attribute['value'])
                                        parent_depth = 1
                                        if match_rule.has_key("parent_depth"):
                                            parent_depth = match_rule["parent_depth"]
                                        tags = [tag for tag in tags if getParentNode(tag, parent_depth).tagName == match_rule['parent_tag'] and getParentNode(tag, parent_depth).getAttribute(parent_attribute_name) == parent_attribute_value]
                                    else:
                                        parent_depth = 1
                                        if match_rule.has_key("parent_depth"):
                                            parent_depth = match_rule["parent_depth"]
                                        tags = [tag for tag in tags if getParentNode(tag, parent_depth).tagName == match_rule['parent_tag'] and getParentNode(tag, parent_depth).hasAttribute(parent_attribute_name)]
                            else:
                                tags = [tag for tag in tags if getParentNode(tag, parent_depth).tagName == match_rule['parent_tag']]

                        target_tags = tags

                    else:
                        raise Exception("yaml does not have tag section")

                    if len(target_tags) > 0:
                        for tag in target_tags:
                            parent = tag.parentNode
                            # remove the tag
                            if replace_xml.has_key('replaced_xml'):
                                parent.removeChild(tag)
                                if str(replace_xml['replaced_xml']) != "":
                                    parent.appendChild(parseString(str(replace_xml['replaced_xml'])).documentElement)
                            elif replace_xml.has_key('replaced_attribute_name') and replace_xml.has_key('replaced_attribute_value'):
                                tag.setAttribute(str(replace_xml['replaced_attribute_name']),
                                                 str(replace_xml['replaced_attribute_value']))
                            elif replace_xml.has_key('added_xml'):
                                if str(replace_xml['added_xml']) != "":
                                    tag.appendChild(parseString(str(replace_xml['added_xml'])).documentElement)
                            else:
                                raise Exception("No rule to replacement is specified")
        else:
            sys.stderr.write('no configuration file !\n')

        if args.output:
            f = open(args.output, 'wb')
            f.write(obj.toprettyxml('\t'))
            f.close()
        else:
            sys.stdout.write(obj.toprettyxml('\t')) ## wirting to standard output

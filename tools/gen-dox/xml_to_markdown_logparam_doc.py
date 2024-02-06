import xml.etree.ElementTree as ET
import json
import sys
import os

# Runs this directly from the crazyflie firmware directory with python tools/gen-dox/xmlparser
# Make sure there are XML docs to parse! check the readme



def create_log_markdown(xml_dir, api_doc_dir, file_name):
    log_groups = parse_xml('logs', xml_dir)
    create_markdown(file_name, log_groups, api_doc_dir)


def create_param_markdown(xml_dir, api_doc_dir, file_name):
    param_groups = parse_xml('params', xml_dir)
    create_markdown(file_name, param_groups, api_doc_dir)


def read_and_parse_xml(file_name):
    with open(file_name, 'rt') as file:
        xml = file.read()
        pre_processed_xml = pre_process_xml(xml)
        return ET.fromstring(pre_processed_xml)


def pre_process_xml(xml):
    # The xml we get from doxygen contains some un-orthodox elements that we handle before we parse the xml
    itemized_result = xml.find('<itemizedlist>')
    if itemized_result != -1:
        related_text = extract_related_text(xml, itemized_result, 50, 100)
        raise ValueError("The xml contains <itemizedlist> which is not supported in log/param documentation. It is probably caused by a list created using '-'. Excerpt: >>>" + related_text + "<<<")

    ulink_result = xml.find('<ulink')
    if ulink_result != -1:
        related_text = extract_related_text(xml, ulink_result, 50, 100)
        raise ValueError("The xml contains <ulink> which is not supported in log/param documentation. It is probably caused by a link, please prepend the URL with '%'. Excerpt: >>>" + related_text + "<<<")

    return xml.replace('<linebreak/>', "")

def extract_related_text(xml, position, pre, post):
    start = max(0, position - pre)
    end = position + post
    return xml[start:end]

def merge_paras(paras, separator):
    parts = []
    if paras is not None:
        for para in paras:
            if para.text is not None:
                parts.append(para.text)

    return separator.join(parts)


def get_brief_description(element):
    paras = element.findall('briefdescription/para')
    return merge_paras(paras, ' ')


def get_detailed_description(element):
    paras = element.findall('detaileddescription/para')
    return merge_paras(paras, '\n\n')


def parse_xml(doc_type, xml_dir):
    groups_info_storage = []
    search_string = ''
    if doc_type == 'logs':
        search_string = 'fake__log__class__'
        replace_string = 'fake_log_class_'
        core_string = 'LOG__CORE__GROUP'
    elif doc_type == 'params':
        search_string = 'fake__param__class__'
        replace_string = 'fake_param_class_'
        core_string = 'PARAM__CORE__GROUP'
    else:
        print('group type does not exist!')
        return None

    root = read_and_parse_xml(os.path.join(xml_dir, 'index.xml'))

    for compound in root.findall('compound'):
        filename = compound.attrib['refid']

        # Go through all the log classes
        if search_string in filename:
            full_filename = filename + '.xml'
            full_group_info = process_class_file(os.path.join(xml_dir, full_filename), replace_string, core_string)
            groups_info_storage.append(full_group_info)

    return groups_info_storage


def process_class_file(file_name, replace_string, core_string):
    root = read_and_parse_xml(file_name)

    # Group name
    for compounddef in root.findall('.//compounddef'):
        compoundname = compounddef.find('compoundname')
        group_name_fake = compoundname.text
        group_name = group_name_fake.replace(replace_string, '')
        group_description = get_detailed_description(compounddef)

    group_info = [group_name, group_description]

    info_variables = extract_memberdefs(root, core_string)
    return [group_info, info_variables]


def extract_memberdefs(root, core_string):
    info_variables = []

    for memberdef in root.findall('.//memberdef'):
        variable_name = memberdef.find('name').text

        id_definition = memberdef.attrib['id']
        is_variable_core = False
        if core_string in id_definition:
            is_variable_core = True

        brief_description = get_brief_description(memberdef)
        detaild_description = get_detailed_description(memberdef)

        type_refs_texts = map(lambda type_ref: type_ref.text, memberdef.findall('type/ref'))
        type_variable = ", ".join(type_refs_texts)

        # location and line
        location = memberdef.find('location')
        file_location = location.attrib['file']
        line_location = location.attrib['line']

        info_variable = [variable_name, is_variable_core, brief_description,
                         type_variable, file_location, line_location, detaild_description]
        info_variables.append(info_variable)

    return info_variables


def create_markdown(file_name, groups_info_storage, api_doc_dir):
    full_path = os.path.join(api_doc_dir, file_name)
    f = open(full_path, 'w')

    f.write('## Index\n\n')
    first_letter = ''

    for full_group_info in groups_info_storage:
        group_info = full_group_info[0]
        group_name = group_info[0]
        if first_letter != group_name[0]:
            first_letter = group_name[0]
            f.write('\n')
            f.write('### ' + first_letter + '\n')

        f.write('* [' + group_name + '](#' + group_name.lower() + ')\n')

    for full_group_info in groups_info_storage:
        group_info = full_group_info[0]
        group_name = group_info[0]
        f.write('\n---\n')

        f.write('[back to group index](#index) \n\n')
        f.write('## ' + group_name + '\n')

        group_description = group_info[1]
        f.write(group_description + '\n')

        f.write('### Variables\n\n')

        info_variables = full_group_info[1]
        string_detailed_descriptions = ''

        table_start_string = (' | Name | Core | Type | Description | \n' + ' | ------------- | ------------- | ----- |----- |\n')
        f.write(table_start_string)

        for info_variable in info_variables:
            variable_name = info_variable[0]
            is_variable_core = info_variable[1]
            brief_description = info_variable[2]
            type_variable = info_variable[3]
            file_location = info_variable[4]
            line_location = info_variable[5]
            detailed_description = info_variable[6]

            if detailed_description is None:
                detailed_description = ''

            string_variable_info = ''
            full_name = group_name + '.' + variable_name
            full_name_nodot = group_name + variable_name
            full_name_url = '[' + full_name + '](#' + full_name_nodot.lower() + ')'

            core_indication_string = ''
            if is_variable_core:
                core_indication_string = 'Core'

            string_variable_info = (' | ' + full_name_url + ' | ' + core_indication_string + ' | ' + type_variable + ' |' + brief_description + ' |\n')
            f.write(string_variable_info)

            if len(brief_description) > 0:
                brief_description = '*' + brief_description.rstrip() + '*'

            string_detailed_description = ('#### **' + full_name + '**\n\n ' +
                '[' + file_location + ' (L' + line_location + ')](https://github.com/bitcraze/crazyflie-firmware/blob/master/' + file_location + '#L' + line_location + ')\n\n' +
                brief_description + '\n\n' +
                detailed_description + '\n\n')

            string_detailed_descriptions += string_detailed_description

        f.write('\n')
        f.write('### Detailed Variable Information\n')

        f.write(string_detailed_descriptions)

    f.close()


def create_json(xml_dir: str, api_doc_dir: str):
    groups = dict()
    groups['params'] = parse_xml('params', xml_dir)
    groups['logs'] = parse_xml('logs', xml_dir)

    json_file = os.path.join(api_doc_dir, 'log_param_doc.json')
    with open(json_file, 'w') as f:
        json_out = dict()
        for key, group in groups.items():
            json_out[key] = dict()

            for info in group:
                group_info = info[0]
                json_out[key][group_info[0]] = {
                    'desc': group_info[1],
                    'variables': dict((var[0], {
                        'core': var[1],
                        'short_desc': var[2],
                        'type': var[3],
                        'desc': var[6]
                    }) for var in info[1])
                }
        f.write(json.dumps(json_out))

if __name__ == '__main__':

    if(len(sys.argv) != 5):
        raise ValueError("Need four arguments!")

    xml_dir = sys.argv[1]
    api_doc_dir = sys.argv[2]
    logs_md_file_name = sys.argv[3]
    params_md_file_name = sys.argv[4]

    print('Create Logging API Markdown files')
    create_log_markdown(xml_dir, api_doc_dir, logs_md_file_name)
    print('Create Parameter API Markdown files')
    create_param_markdown(xml_dir, api_doc_dir, params_md_file_name)

    print('Create JSON file')
    # Note: The json file is used by the python client build
    create_json(xml_dir, api_doc_dir)

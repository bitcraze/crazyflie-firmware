import xml.etree.ElementTree as ET

# Runs this directly from the crazyflie firmware directory with python tools/gen-dox/xmlparser
# Make sure there are XML docs to parse! check the readme


def create_log_markdown():
    log_groups = parse_xml('logs')
    create_markdown('logs', log_groups)


def create_param_markdown():
    param_groups = parse_xml('params')
    create_markdown('params', param_groups)

def parse_xml(doc_type):

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


    # Get the index file
    tree = ET.parse('generated/dox/xml/index.xml')
    root = tree.getroot()

    for compound in root.findall('compound'):
        filename = compound.attrib['refid']
        
        #Go through all the log classes in 
        if search_string in filename:
            full_filename = filename + '.xml'
            tree = ET.parse('generated/dox/xml/' + full_filename)
            root = tree.getroot()

            # log group name
            for compounddef in root.findall('.//compounddef'):
                compoundname = compounddef.find('compoundname')
                group_name_fake = compoundname.text
                group_name = group_name_fake.replace(replace_string,'')
                descrp = compounddef.find('detaileddescription/para')
                if descrp != None:
                    group_description = descrp.text
                else:
                    group_description = ''


            group_info = [group_name, group_description]

            info_variables = []
            # Variable name
            for memberdef in root.findall('.//memberdef'):
                #name
                name = memberdef.find('name')
                variable_name = name.text
                id_definition = memberdef.attrib['id']
                is_variable_core = False
                if core_string in id_definition:
                    is_variable_core = True
                #description
                descrp = memberdef.find('briefdescription/para')
                if descrp != None:
                    description = descrp.text
                else:
                    description = ''
                #type
                typevar = memberdef.find('type/ref')
                type_variable = typevar.text;
                #location and line
                location = memberdef.find('location')
                file_location =location.attrib['file']
                line_location =location.attrib['line']


                info_variable = [variable_name, is_variable_core, description, type_variable,file_location, line_location]
                info_variables.append(info_variable)

            full_group_info = [group_info, info_variables]
            
            groups_info_storage.append(full_group_info)

    return groups_info_storage


def create_markdown(doc_type, groups_info_storage):

    markdown_file_name = doc_type + '.md'
    f = open(markdown_file_name, 'w')
    if doc_type == 'logs':
        f.write('# Logging groups and variables\n')
    elif doc_type == 'params':
        f.write('# Parameter groups and variables\n')
    else:
        print('group type does not exist!')
        return None
        
    f.write('## Index\n')
    first_letter = ''

    for full_group_info in groups_info_storage:
        group_info = full_group_info[0]
        group_name = group_info[0]
        if first_letter != group_name[0]:
            first_letter = group_name[0]
            f.write('## ' + first_letter + '\n')

        f.write('* [' +group_name+'](#'+group_name+')\n')

    for full_group_info in groups_info_storage:
        group_info = full_group_info[0]
        group_name = group_info[0]
        f.write('\n---\n')

        f.write('[back to log index](#index) \n\n')
        f.write('## '+group_name + '\n')

        group_description = group_info[1]
        f.write(group_description + '\n')

        f.write('### Core Log variables\n')

        info_variables = full_group_info[1]
        string_dev_variable = ''
        dev_variables = []
        core_variable_exist_in_group = False
        dev_variables_exist_in_group= False
        for info_variable in info_variables:
            variable_name = info_variable[0] 
            is_variable_core = info_variable[1] 
            description = info_variable[2] 
            type_variable= info_variable[3] 
            file_location= info_variable[4] 
            line_location= info_variable[5]

            string_variable_info = ('* ' +  type_variable + ' **' + variable_name + '** \n' + 
            '   * ' + description+ '\n' + 
            '   * [' + file_location + ' (L'+line_location+')](https://github.com/bitcraze/crazyflie-firmware/blob/master/' + 
            file_location +'#L'+line_location+')\n')

            if is_variable_core:
                f.write(string_variable_info)
                core_variable_exist_in_group= True
            else:
                string_dev_variable = string_dev_variable + string_variable_info
                dev_variables_exist_in_group= True

        if core_variable_exist_in_group== False:
            f.write('### *No core log variables* \n')

        if dev_variables_exist_in_group:
            f.write('### Dev log variables\n')
            f.write(string_dev_variable)
        
    f.close()

if __name__ == '__main__':

    print('Create Logging API Markdown files')
    create_log_markdown()
    print('Create Parameter API Markdown files')
    create_param_markdown()

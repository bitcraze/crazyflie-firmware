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


            #input file
    fin = open("generated/dox/xml/index.xml", "rt")
    
    #fout = open("generated/dox/xml/index.xml", "wt")
    #for line in fin:
    #fout.write(line.replace('<linebreak/>', '\n'))
    #fin.close()
    #fout.close()


    # Get the index file
    root = ET.fromstring(fin.read())
    #tree = ET.parse('generated/dox/xml/index.xml')
    #root = tree.getroot()

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
                descrp = compounddef.findall('detaileddescription/para')
                group_description = ''
                if descrp != None:
                    for para in descrp:
                        if para.text != None:
                            group_description = group_description + '\n\n' + para.text

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

                big_descrp = memberdef.findall('detaileddescription/para')
                big_description = ''
                if big_descrp != None:
                    for para in big_descrp:
                        if para.text != None:
                            big_description = big_description + '\n\n' + para.text
                #type
                typevar = memberdef.find('type/ref')
                type_variable = typevar.text;
                #location and line
                location = memberdef.find('location')
                file_location =location.attrib['file']
                line_location =location.attrib['line']


                info_variable = [variable_name, is_variable_core, description, type_variable,file_location, line_location, big_description]
                info_variables.append(info_variable)

            full_group_info = [group_info, info_variables]
            
            groups_info_storage.append(full_group_info)

    return groups_info_storage


def create_markdown(doc_type, groups_info_storage):

    markdown_file_name = 'docs/api/'+doc_type + '.md'
    f = open(markdown_file_name, 'w')

    f.write('---\n')
    if doc_type == 'logs':
        f.write('title: Logging groups and variables\n')
    elif doc_type == 'params':
        f.write('title: Parameter groups and variables\n')
    else:
        print('group type does not exist!')
        return None

    f.write('page_id: ' +doc_type + '\n')
    f.write('---\n')

        
    f.write('## Index\n\n')
    first_letter = ''

    for full_group_info in groups_info_storage:
        group_info = full_group_info[0]
        group_name = group_info[0]
        if first_letter != group_name[0]:
            first_letter = group_name[0]
            f.write('\n')
            f.write('### ' + first_letter + '\n')

        f.write('* [' +group_name+'](#'+group_name.lower()+')\n')

    for full_group_info in groups_info_storage:
        group_info = full_group_info[0]
        group_name = group_info[0]
        f.write('\n---\n')

        f.write('[back to group index](#index) \n\n')
        f.write('## '+group_name + '\n')

        group_description = group_info[1]
        f.write(group_description + '\n')

        f.write('### Variables\n\n')

        info_variables = full_group_info[1]
        string_big_descriptions = ''
        dev_variables = []
        core_variable_exist_in_group = False
        dev_variables_exist_in_group= False

        table_start_string = (' | Name | Core | Type | Description | \n' +
        ' | ------------- | ------------- | ----- |----- |\n')
        f.write(table_start_string)

        for info_variable in info_variables:
            variable_name = info_variable[0] 
            is_variable_core = info_variable[1] 
            description = info_variable[2] 
            type_variable= info_variable[3] 
            file_location= info_variable[4] 
            line_location= info_variable[5]
            big_description = info_variable[6]

            string_variable_info = ''
            full_name = group_name + '.' + variable_name
            full_name_nodot =  group_name + variable_name
            full_name_url = '['+full_name+'](#'+full_name_nodot.lower()+')'


            core_indication_string = ''
            if is_variable_core:
                core_indication_string = 'Core'

            string_variable_info = (' | '+full_name_url+' | '+ core_indication_string+ ' | ' + type_variable + ' |'
             + description + ' |\n')
            f.write(string_variable_info)


            if big_description == None:
                big_description = ''

            
            if len(description) >0:
                description = '*'+description.rstrip()+'*'

            
    

            string_big_description = ('#### **'+ full_name + '**\n \n ' +
            '[' + file_location + ' (L'+line_location+')](https://github.com/bitcraze/crazyflie-firmware/blob/master/' + 
            file_location +'#L'+line_location+')\n \n' +
            description + '\n\n'+
            big_description+'\n\n')

            string_big_descriptions = string_big_descriptions +string_big_description



            '''
            if len(description)>0:
                string_variable_info = ('* ' +  type_variable + ' **' + variable_name + '** \n' + 
                '   * ' + description+ '\n' + 
                '   * [' + file_location + ' (L'+line_location+')](https://github.com/bitcraze/crazyflie-firmware/blob/master/' + 
                file_location +'#L'+line_location+')\n')
            else:
                string_variable_info = ('* ' +  type_variable + ' **' + variable_name + '** \n' + 
                '   * [' + file_location + ' (L'+line_location+')](https://github.com/bitcraze/crazyflie-firmware/blob/master/' + 
                file_location +'#L'+line_location+')\n')

            if is_variable_core:
                f.write(string_variable_info)
                core_variable_exist_in_group= True
            else:
                string_dev_variable = string_dev_variable + string_variable_info
                dev_variables_exist_in_group= True

        if core_variable_exist_in_group== False:
            f.write(' *No core log variables* \n')

        if dev_variables_exist_in_group:
            f.write('### Dev log variables\n')
            f.write(string_dev_variable)'''
            
        f.write('\n')
        f.write('### Detailed Variable Information\n')  

        f.write(string_big_descriptions)

        
    f.close()

if __name__ == '__main__':

    print('Create Logging API Markdown files')
    create_log_markdown()
    print('Create Parameter API Markdown files')
    create_param_markdown()

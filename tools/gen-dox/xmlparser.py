import xml.etree.ElementTree as ET

# Runs this directly from the crazyflie firmware directory with python tools/gen-dox/xmlparser
# Make sure there are XML docs to parse! check the readme

log_groups = []

# Get the index file
tree = ET.parse('generated/dox/xml/index.xml')
root = tree.getroot()

for compound in root.findall('compound'):
    filename = compound.attrib['refid']
    
    #Go through all the log classes in 
    if 'fake__log__class__' in filename:
        print(filename)
        full_filename = filename + '.xml'
        tree = ET.parse('generated/dox/xml/' + full_filename)
        root = tree.getroot()

        # log group name
        for compounddef in root.findall('.//compounddef'):
            print('LOG GROUP NAME')
            compoundname = compounddef.find('compoundname')
            log_group_name_fake = compoundname.text
            log_group_name = log_group_name_fake.replace('fake_log_class_','')
            descrp = compounddef.find('detaileddescription/para')
            if descrp != None:
                log_group_description = descrp.text
            else:
                log_group_description = ''


        log_group_info = [log_group_name, log_group_description]
        print('LOG VARIABLES')

        log_variables = []
        # Variable name
        for memberdef in root.findall('.//memberdef'):
            #name
            name = memberdef.find('name')
            log_name = name.text
            id_definition = memberdef.attrib['id']
            core = False
            if 'LOG__CORE__GROUP' in id_definition:
                core = True
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
            # core variable


            log_variable = [log_name, core, description, type_variable,file_location, line_location]
            log_variables.append(log_variable)
            print(log_variable)

        full_log_group_info = [log_group_info, log_variables]
        print(full_log_group_info)
        
        log_groups.append(full_log_group_info)

f = open("logs.md", "w")
f.write('# Logging groups and variables\n')
f.write('## Index\n')
first_letter = ''

for full_log_group_info in log_groups:
    log_group_info = full_log_group_info[0]
    log_group_name = log_group_info[0]
    if first_letter != log_group_name[0]:
        first_letter = log_group_name[0]
        f.write('## ' + first_letter + '\n')

    f.write('* [' +log_group_name+'](#'+log_group_name+')\n')

for full_log_group_info in log_groups:
    log_group_info = full_log_group_info[0]
    log_group_name = log_group_info[0]
    f.write('\n---\n')

    f.write('[back to log index](#index) \n\n')
    f.write('## '+log_group_name + '\n')

    log_group_description = log_group_info[1]
    f.write(log_group_description + '\n')

    f.write('### Core Log variables\n')

    log_variables = full_log_group_info[1]
    string_dev_variable = ''
    dev_variables = []
    core_variable_exist = False
    dev_variables_exist = False
    for log_variable in log_variables:
        print(log_variable)
        log_name = log_variable[0] 
        core = log_variable[1] 
        description = log_variable[2] 
        type_variable= log_variable[3] 
        file_location= log_variable[4] 
        line_location= log_variable[5]

        print(type(type_variable), type(log_name))
        string_variable_info = ('* ' +  type_variable + ' **' + log_name + '** \n' + 
        '   * ' + description+ '\n' + 
        '   * [' + file_location + ' (L'+line_location+')](https://github.com/bitcraze/crazyflie-firmware/blob/master/' + 
        file_location +'#L'+line_location+')\n')

        if core:
            f.write(string_variable_info)
            core_variable_exist = True
        else:
            string_dev_variable = string_dev_variable + string_variable_info
            dev_variables_exist = True

    if core_variable_exist == False:
        f.write('### *No core log variables* \n')

    if dev_variables_exist:
        f.write('### Dev log variables\n')
        f.write(string_dev_variable)
    
f.close()

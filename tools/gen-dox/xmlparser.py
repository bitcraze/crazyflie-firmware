import xml.etree.ElementTree as ET

# Runs this directly from the crazyflie firmware directory with python tools/gen-dox/xmlparser
# Make sure there are XML docs to parse! check the readme

## To store the info from the xml file
class logStruct:
    pass

logStruct.name = []

# Get the index file
tree = ET.parse('generated-docs/xml/index.xml')
root = tree.getroot()

for compound in root.findall('compound'):
    filename = compound.attrib['refid']
    if 'fake__log__class__' in filename:
        print(filename)
        full_filename = filename + '.xml'
        tree = ET.parse('generated-docs/xml/' + full_filename)
        root = tree.getroot()

        # log group name
        for compounddef in root.findall('.//compounddef'):
            print('LOG GROUP NAME')
            compoundname = compounddef.find('compoundname')
            print(compoundname.text)
            descrp = compounddef.find('detaileddescription/para')
            if descrp != None:
                print(descrp.text)

        logStruct.name.append(compoundname)

        print('LOG VARIABLES')

        # Variable name
        for memberdef in root.findall('.//memberdef'):
            #name
            name = memberdef.find('name')
            print(name.text)
            #description
            descrp = memberdef.find('briefdescription/para')
            if descrp != None:
                print(descrp.text)
            #type
            typevar = memberdef.find('type/ref')
            print(typevar.text)
            #location and line
            location = memberdef.find('location')
            print(location.attrib['file'])
            print(location.attrib['line'])




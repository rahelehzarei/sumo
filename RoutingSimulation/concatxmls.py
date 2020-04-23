import os, os.path, sys
import glob
from xml.etree import ElementTree
import ntpath
from xml.dom import minidom

dirpath = os.getcwd()
algoPath = 'RoutingSim'
#algo =sys.argv[1]
TripsNo =["20", "30", "40", "50", "60", "70", "80", "90", "100", "110" , "120", "130", "140", "150", "160", "170", "180", "190", "200"]

for i in range(1,10):
    Trips = ElementTree.parse(
        os.path.dirname(dirpath + '/' + 'Trips10/' + 'RoutingSim'+ str(i)+'.rou.xml') + '/' + os.path.basename(
            dirpath + '/' + 'Trips10/' + 'RoutingSim' +str(i)+'.rou.xml'))
    tripsToInsert = (Trips.getroot()).findall('./trip')
    xml_files = []
    for j in TripsNo:
        xml_files.append(glob.glob(dirpath + '/' + 'Trips'+j +'/'+ 'RoutingSim'+ str(i) +'.rou.xml'))

    for file in xml_files:
        my_file = open(os.path.dirname(file[0])+'/'+os.path.basename(file[0]), "r")
        lines_of_file = my_file.readlines()
        my_file.close()
        i = 10
        for t in tripsToInsert:
            lines_of_file.insert(-1, "    <trip id="'"' +str(i)+ '"' "  depart="'"' +str(i)+'"' "  from=" '"' +str(t.attrib['from'])+ '"' "  to=" '"' +str(t.attrib['to'])+ '"' " />  \n")
            i = i+1
            outFile = open(os.path.dirname(file[0])+'/'+os.path.basename(file[0]), 'w')
            outFile.writelines(lines_of_file)
            outFile.close()
            # my_file.writelines(lines_of_file)

        et = ElementTree.parse(os.path.dirname(file[0])+'/'+os.path.basename(file[0]))


        # i = 9
        # for t in tripsToInsert:
        #     new_tag = ElementTree.SubElement(et.getroot(), 'trip')
        #     new_tag.attrib = t.attrib
        #     new_tag.attrib['id'] = str(i+1)
        #     new_tag.attrib['depart'] = i+1
        #     i = i+1
        #     new_tag.tail = t.tail
        #     new_tag.text = t.text
        # et.write(os.path.dirname(file[0])+'/'+os.path.basename(file[0]))

        tripsToInsert = et.getroot().findall("./trip")
              # tripsToInsert.append(trip)
        # for result in data.iter('routes'):
        # if xml_element_tree is None:
        # xml_element_tree = root
          # root = ElementTree.fromstring(file)
        #
        # for trip in root.findall("./trip"):
        #     # root.insert(1, trip)
        #     tripsToInsert.append(trip)
        # insertion_point = root.findall("./trip")[9]



        # ElementTree.dump(root)


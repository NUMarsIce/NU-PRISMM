import csv
import rosbag
name = raw_input("Enter bag name: ")
bag = rosbag.Bag(name)

with open('data_pas.csv', 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    #spamwriter.writerow(["time", "loadA", "loadB", "drill_current", "power_current", "pow24_current", "pow5_current"])
    for topic, msg, t in bag.read_messages(topics=["/pas_data"]):
        spamwriter.writerow([t.secs+t.nsecs/10e9, msg.loadA, msg.loadB, msg.drill_current, msg.power_current, msg.pow24_current, msg.pow5_current])

with open('data_dam.csv', 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)

    #spamwriter.writerow(["stp_drill", "drill_stp_current"])
    for topic, msg, t in bag.read_messages(topics=["/dam_data"]):
        spamwriter.writerow([t.secs+t.nsecs/10e9, msg.stp_drill, msg.drill_stp_current])
bag.close()

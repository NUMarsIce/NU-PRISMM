import rosbag
bag = rosbag.Bag(input("Enter bag name"))
for topic, msg, t in bag.read_messages(topics=['pas_data']):
    print(msg)
bag.close()

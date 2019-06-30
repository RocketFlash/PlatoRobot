import rosbag


def change_frame_id(inbag, outbag, frame_id, topics):
    outbag = rosbag.Bag(outbag, 'w')
    for topic, msg, t in rosbag.Bag(inbag, 'r').read_messages():
        if topic in topics:
            if msg._has_header:
                msg.header.frame_id = frame_id
        outbag.write(topic, msg, t)
    outbag.close()


bag_files_path = '../../plato_navigation/bagfiles/'
bagfile_name_hololens = 'hololens.bag'
bagfile_name_hololens_new = 'hololens_new.bag'
topic_name_hololens = '/pointCloud2Test'

change_frame_id('{}{}'.format(bag_files_path, bagfile_name_hololens),
                '{}{}'.format(bag_files_path, bagfile_name_hololens_new),
                'holo', topic_name_hololens)

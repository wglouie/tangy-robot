#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <keyboard/Key.h>
#include <fault_diagnosis/AudioData.h>
#include <text_to_speech/tts.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

class RosGstCapture{
public:
    RosGstCapture(std::string destination) //filename for file or appsink for stream
    {
        _bitrate = 192;
        std::string dst_type = destination;

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);

        // The destination of the audio
        //ros::param::param<std::string>("~dst", dst_type, "appsink");

        _pub = _nh.advertise<fault_diagnosis::AudioData>("audio", 10);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        if (dst_type == "appsink")
        {
            _sink = gst_element_factory_make("appsink", "sink");
            g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
            g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
            g_signal_connect( G_OBJECT(_sink), "new-sample",
                              G_CALLBACK(onNewBuffer), this);
        }
        else
        {
            _sink = gst_element_factory_make("filesink", "sink");
            g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        _source = gst_element_factory_make("alsasrc", "source");
        _convert = gst_element_factory_make("audioconvert", "convert");

        gboolean link_ok;

        if (_format == "mp3"){
            _encode = gst_element_factory_make("lamemp3enc", "encoder");
            g_object_set( G_OBJECT(_encode), "quality", 2.0, NULL);
            g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);

            gst_bin_add_many( GST_BIN(_pipeline), _source, _convert, _encode, _sink, NULL);
            link_ok = gst_element_link_many(_source, _convert, _encode, _sink, NULL);
        } else {
            //ROS_ERROR_STREAM("format must be \"mp3\"");
            exitOnMainThread(1);
        }

        if (!link_ok) {
            //ROS_ERROR_STREAM("Unsupported media type.");
            exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread(boost::bind(g_main_loop_run, _loop));
    }

    ~RosGstCapture()
    {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
    }

    void exitOnMainThread(int code)
    {
        exit(code);
    }

    void publish( const fault_diagnosis::AudioData &msg )
    {
        _pub.publish(msg);
    }

    static GstFlowReturn onNewBuffer (GstAppSink *appsink, gpointer userData)
    {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        fault_diagnosis::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy( &msg.data[0], map.data, map.size );

        server->publish(msg);

        return GST_FLOW_OK;
    }

    static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
    {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        //ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;

    boost::thread _gst_thread;

    GstElement *_pipeline, *_source, *_sink, *_convert, *_encode;
    GstBus *_bus;
    int _bitrate;
    GMainLoop *_loop;
    std::string _format = "mp3";
};


class AudioSaver{
public:
    AudioSaver(int no) : count(no){
        key = n.subscribe("/keyboard/keydown", 1, &AudioSaver::Save, this);
        //audsub = it.subscribe("audio", 1, &AudioSaver::AudCb, this);
        client = n.serviceClient<text_to_speech::tts>("tts");
        read_file("testlines.txt");
        myfile.open("/home/tangy/audio_classification.txt");
    }

private:
    ros::NodeHandle n;
    ros::Subscriber key;
    std::ofstream myfile;
    ros::ServiceClient client;
    //ros::Subscriber audsub;
    int count;
    std::vector<std::string> list_of_lines;

    void Save(const keyboard::Key::ConstPtr& msg){
        //on keydown, we want to play and record simultaneously

        if(msg->code == 48 || msg->code == 49){
            std::string filename = std::to_string(count) + ".mp3";
            RosGstCapture server(filename); //record using audio_capture and pause to let it spin up
            ros::Duration(1).sleep();
            //play using tts service if one
            if(msg->code == 49) {
                text_to_speech::tts srv;
                srv.request.speak = get_rand_line();
                srv.request.gender = std::rand() % 2;// == 0 ? text_to_speech::tts::Request::MALE : text_to_speech::tts::Request::FEMALE;
                srv.request.language = "en";
                srv.request.webbased = true;
                if(!client.call(srv)){
                    std::cout << "Error: " + srv.response.error << std::endl;
                }
            } else { //if no speech then just delay a short time to record
                ros::Duration(rand()%8).sleep();
            }
            myfile << filename << (msg->code == 48 ? " 0" : " 1") << std::endl;
            std::cout << "Saved audio file " << filename << std::endl;
            count++;
        }
    }

    void read_file(std::string filePath)
    {

        std::ifstream smallTalkStream(filePath.c_str());
        if (smallTalkStream.is_open()){
            std::string l;
            while(getline(smallTalkStream, l)){
                if(!l.empty()){
                    list_of_lines.push_back(l);
                }
            }
            smallTalkStream.close();
        }
    }

    std::string get_rand_line()
    {
        if(!list_of_lines.empty()){
            int rand_n = std::rand()%(list_of_lines.size());
            return list_of_lines.at(rand_n);
        }
        return "";
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "audio_saver");
    AudioSaver saver(std::atoi(argv[0]));
    gst_init(&argc, &argv);

    ros::spin();

    return 0;
}

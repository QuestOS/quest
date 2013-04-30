#ifndef FFSERVER_CONF_H
#define FFSERVER_CONF_H

char* FFSERVER_CONF = "Port 8090\
BindAddress 192.168.2.11 \
MaxHTTPConnections 2000\
MaxClients 1000\
MaxBandwidth 1000\
CustomLog -\
<Feed feed1.ffm>\
File /tmp/feed1.ffm\
FileMaxSize 200K\
ACL allow 192.168.2.1 \
</Feed>\
<Stream test1.mpg>\
Feed feed1.ffm\
Format mpeg\
AudioBitRate 32\
AudioChannels 1\
AudioSampleRate 44100\
VideoBitRate 64\
VideoBufferSize 40\
VideoFrameRate 3\
VideoSize 160x128\
VideoGopSize 12\
</Stream>\
<Stream test.asf>\
Feed feed1.ffm\
Format asf\
VideoFrameRate 15\
VideoSize 352x240\
VideoBitRate 256\
VideoBufferSize 40\
VideoGopSize 30\
AudioBitRate 64\
StartSendOnKey\
</Stream>\
<Stream stat.html>\
Format status\
ACL allow localhost\
ACL allow 192.168.0.0 192.168.255.255\
</Stream>\
<Redirect index.html>\
URL http://www.ffmpeg.org/\
</Redirect>"

#endif

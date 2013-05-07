#ifndef FFSERVER_CONF_H
#define FFSERVER_CONF_H

char* FFSERVER_CONF = "Port 8090\n\
BindAddress 192.168.2.11\n\
\MaxHTTPConnections 2000\n\
MaxClients 1000\n\
MaxBandwidth 1000\n\
CustomLog -\n\
#<Feed feed1.ffm>\n\
#File /tmp/feed1.ffm\n\
#FileMaxSize 200K\n\
#ACL allow 192.168.2.1 \n\
#</Feed>\n\
#<Stream test1.ffm>\n\
#Feed feed1.ffm\n\
#Format ffm\n\
#AudioBitRate 32\n\
#AudioChannels 1\n\
#AudioSampleRate 44100\n\
#VideoBitRate 64\n\
#VideoBufferSize 40\n\
#VideoFrameRate 3\n\
#VideoSize 160x128\n\
#VideoGopSize 12\n\
#</Stream>\n\
#<Stream test2.ffm>\n\
#Feed feed2.ffm\n\
#Format ffm\n\
#VideoFrameRate 15\n\
#VideoSize 352x240\n\
#VideoBitRate 256\n\
#VideoBufferSize 40\n\
#VideoGopSize 30\n\
#AudioBitRate 64\n\
#StartSendOnKey\n\
#</Stream>\n\
<Stream stat.html>\n\
Format status\n\
#ACL allow localhost\n\
#ACL allow 192.168.0.0 192.168.255.255\n\
</Stream>\n\
<Redirect index.html>\n\
URL http://www.ffmpeg.org/\n\
</Redirect>";
#endif

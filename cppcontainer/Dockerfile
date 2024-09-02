FROM opensuse/leap:15.6

RUN zypper refresh 
RUN zypper install -y gcc12-c++
RUN mkdir /work
WORKDIR /work

CMD ["/usr/bin/bash","start.sh"]


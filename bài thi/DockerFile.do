FROM ubuntu:18.04
RUN apt update -y \
    && apt upgrade -y
RUN apt install -y build-essential wget zlib1g-dev libncurses5-dev libgdb-dev libnss3-dev libssl-dev libreadline-dev 
RUN wget https://www.python.org/ftp/python/3.7.6/Python-3.7.6.tgz \
    && tar -xf Python-3.7.6.thz \
    && cd Python-3.7.6 \
    && ./configure --enable-optimizations \
    && make -j 8 \
    && make altinstall \
    && cd


RUN apt install python3-pip -y \
    && python3.7 -m pip install --upgrade pip

WORKDIR /install
COPY requirements.txt .

RUN chmod +x requirements.txt \
    && ./requirements.txt -y \

RUN apt install libgl1-mesa-glx -y \
    && apt-get install libgtk2.0-0 -y \
    && apt install -y libsm6 -y \

RUN apt install git -y \
    && cd \
    && git clone https://github.com/phonghongs/Unity-UIT_Car.git


CMD python3.7 ~/Unity-UIT_Car/Code\ test\ Simulation/Raw\ code/raw_code.py

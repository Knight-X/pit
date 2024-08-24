FROM python:3.10.14-slim-bookworm


WORKDIR /app

RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip
RUN pip install numpy==1.26.4
RUN pip install ns3 gymnasium stable_baselines3[extra] opencv-python-headless

RUN apt-get update && apt-get install -y ffmpeg libsm6 libxext6 
RUN apt-get install -y vim tmux

CMD ["tail", "-f", "/dev/null"] 

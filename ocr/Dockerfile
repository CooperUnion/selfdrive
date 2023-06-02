# Use the Python 3.8.16 slim-buster base image
FROM python:3.8.16-slim-buster

# Set the working directory inside the container
WORKDIR /app

# Install Git and necessary dependencies
RUN apt-get update && apt-get install -y git tesseract-ocr libgl1-mesa-glx ffmpeg libsm6 libxext6 vim

# Clone the pytesseract repository
RUN git clone https://github.com/madmaze/pytesseract.git

# Change directory to pytesseract and install it
WORKDIR /app/pytesseract
RUN pip install -U .

# Change back to the /app directory
WORKDIR /app

# Copy the ocr.py file into the container at /app
# COPY ocr.py /app/ocr.py
# COPY utils.py /app/utils.py

# Install any necessary dependencies
# If you have a requirements.txt file, uncomment the following line
COPY requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Set TESSDATA_PREFIX environment variable
ENV TESSDATA_PREFIX '/app/ocr/data/tessdata'

# Define a volume to mount the local directory /home/igvc/data to /app/data inside the container
VOLUME ["/home/igvc/selfdrive/ocr/data", "/app/ocr/data"]
VOLUME ["/home/igvc/selfdrive/ocr", "/app/ocr"]

# Set the entrypoint to run the ocr.py script
# ENTRYPOINT ["python", "ocr.py"]

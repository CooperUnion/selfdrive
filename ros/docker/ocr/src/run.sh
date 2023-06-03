docker build -t ocr .
docker run -v /home/igvc/selfdrive/ocr/data:/app/ocr/data -v /home/igvc/selfdrive/ocr:/app/ocr -it --rm --name ocr-container ocr /bin/bash

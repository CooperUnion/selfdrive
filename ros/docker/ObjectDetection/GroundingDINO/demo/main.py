# import groundingdino.datasets.transforms as transform
from groundingdino.datasets import transforms as transform
from groundingdino.util.inference import load_model, predict, annotate
import groundingdino.datasets.transforms as T
import cv2
from PIL import Image

cap = cv2.VideoCapture(0)
model = load_model("../groundingdino/config/GroundingDINO_SwinT_OGC.py", "../weights/groundingdino_swint_ogc.pth")
#abritrary objects in the room for testing purposes
TEXT_PROMPT = "person . water bottle . phone"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25


while True:
    #probably will be replaced with ZED integration
    ret, frame = cap.read()

    transform = T.Compose(
        [
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ]
    )


    image_source = Image.fromarray(frame).convert("RGB")
    image, _ = transform(image_source, None)

    boxes, logits, phrases = predict(
    model=model,
    image=image,
    caption=TEXT_PROMPT,
    box_threshold=BOX_TRESHOLD,
    text_threshold=TEXT_TRESHOLD)
    #if you want to use cuda make device 'cuda'


    annotated_frame = annotate(
        image_source=frame,
        boxes=boxes,
        logits=logits,
        phrases=phrases
        )

    print(boxes)




    out_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
    cv2.imshow('frame', out_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

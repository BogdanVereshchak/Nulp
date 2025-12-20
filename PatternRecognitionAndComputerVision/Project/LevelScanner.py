import cv2
import numpy as np
import json
import os

# Налаштування кольорів для класифікації (HSV діапазон)
# Це допомагає відрізнити Червоне Коло (Ворог) від Жовтого Кола (Монета)
def get_color_name(hsv_color):
    h = hsv_color[0]
    s = hsv_color[1]
    v = hsv_color[2]
    
    if s < 50: return "white" # Або сірий/чорний
    if h < 10 or h > 170: return "red"
    if 10 < h < 35: return "yellow"
    if 35 < h < 85: return "green"
    if 85 < h < 130: return "blue"
    return "unknown"

def scan_level_image(image_path, output_json_path):
    # 1. Завантаження зображення
    img = cv2.imread(image_path)
    if img is None:
        print("Помилка: Зображення не знайдено.")
        return

    # Зменшуємо шум (розмиття)
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    
    # Переводимо в HSV для роботи з кольором
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # Переводимо в Ч/Б для пошуку форм (Canny edge detection)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY_INV)
    

    # 2. Знаходимо контури
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    level_objects = []

    print(f"Знайдено {len(contours)} об'єктів.")

    for cnt in contours:
        # Ігноруємо дуже дрібні шуми
        area = cv2.contourArea(cnt)
        if area < 500:
            continue

        # 3. АПРОКСИМАЦІЯ (Головний секрет)
        # Це перетворює кривий малюнок на геометричну фігуру
        epsilon = 0.04 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        # Отримуємо координати та розміри
        x, y, w, h = cv2.boundingRect(approx)
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Визначаємо середній колір всередині контуру
        mask = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        mean_val = cv2.mean(hsv, mask=mask)
        color_name = get_color_name(mean_val)
        
        obj_type = "unknown"
        vertices = len(approx)

        # 4. Логіка визначення об'єкта за формою та кольором
        if vertices == 3:
            obj_type = "spikes" # Трикутник
        elif vertices == 4:
            # Перевіряємо співвідношення сторін, щоб відрізнити квадрат від довгої платформи
            aspect_ratio = float(w)/h
            if 0.9 <= aspect_ratio <= 1.1:
                obj_type = "box" # Квадрат
            else:
                obj_type = "platform" # Прямокутник
        else:
            # Якщо вершин багато (> 4) - це коло або складна форма
            if color_name == "yellow":
                obj_type = "coin"
            elif color_name == "red":
                obj_type = "enemy"
            elif color_name == "green":
                obj_type = "player_start"
            elif color_name == "blue":
                obj_type = "finish"
            else:
                obj_type = "Wow" # Просто декорація

        # Додаємо в список
        level_objects.append({
            "type": obj_type,
            "x": int(center_x),
            "y": int(center_y),
            "width": int(w),
            "height": int(h),
            "rotation": 0 # Тут можна додати логіку визначення кута нахилу
        })
        
        # Малюємо для відладки (щоб бачити, що програма зрозуміла)
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(img, obj_type, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 5. Зберігаємо результат
    data = {"objects": level_objects, "level_size": {"w": img.shape[1], "h": img.shape[0]}}
    
    with open(output_json_path, 'w') as f:
        json.dump(data, f, indent=4)
        
    print(f"Рівень збережено в {output_json_path}")
    
    # Показуємо результат аналізу (можна прибрати для фінальної версії)
    cv2.imshow("Analysis Result", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Приклад виклику (це запускатиметься з інтерфейсу)
if __name__ == "__main__":
    # Створи тестовий файл level.png або зміни шлях
    scan_level_image("level_sketch.png", "level_data.json")
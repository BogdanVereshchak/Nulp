const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const {
  FaBrain, FaChartBar, FaGamepad, FaCogs, FaCheckCircle,
  FaLayerGroup, FaDatabase, FaArrowRight, FaTrophy, FaBolt
} = require("react-icons/fa");
const { MdTimeline, MdTune, MdScience } = require("react-icons/md");

async function iconPng(IconComp, color = "#FFFFFF", size = 256) {
  const svg = ReactDOMServer.renderToStaticMarkup(
    React.createElement(IconComp, { color, size: String(size) })
  );
  const buf = await sharp(Buffer.from(svg)).png().toBuffer();
  return "image/png;base64," + buf.toString("base64");
}

async function main() {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.title = "DDA — Система динамічної адаптації складності";

  // ─── Palette ───────────────────────────────────────────────────────────────
  const BG_DARK   = "0D1B2A";   // deep navy — title / conclusion slides
  const BG_LIGHT  = "F4F6FA";   // off-white — content slides
  const ACCENT    = "00C2FF";   // electric cyan
  const ACCENT2   = "7B2FBE";   // violet
  const TEXT_DARK = "1A1A2E";
  const TEXT_MID  = "4A5568";
  const TEXT_LIGHT= "E8EDF5";
  const CARD_BG   = "FFFFFF";

  const makeShadow = () => ({ type: "outer", blur: 8, offset: 3, angle: 135, color: "000000", opacity: 0.10 });

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 1 — TITLE
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_DARK };

    // Geometric accent shapes
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 0.35, h: 5.625, fill: { color: ACCENT }, line: { color: ACCENT } });
    s.addShape(pres.shapes.RECTANGLE, { x: 6.5, y: 0, w: 3.5, h: 5.625, fill: { color: "0A1628", transparency: 20 }, line: { color: "0A1628" } });
    // Accent dots
    for (let i = 0; i < 5; i++) {
      s.addShape(pres.shapes.OVAL, { x: 7.0 + i * 0.55, y: 0.4, w: 0.28, h: 0.28, fill: { color: ACCENT, transparency: 60 + i * 8 }, line: { color: ACCENT, transparency: 60 + i * 8 } });
    }

    s.addText("ПЕРЕДДИПЛОМНА ПРАКТИКА", {
      x: 0.6, y: 0.5, w: 5.7, h: 0.45,
      fontSize: 11, bold: false, color: ACCENT, charSpacing: 3, fontFace: "Calibri"
    });

    s.addText("Система динамічної\nадаптації складності\nу ігровому середовищі", {
      x: 0.6, y: 1.0, w: 5.7, h: 2.5,
      fontSize: 36, bold: true, color: TEXT_LIGHT, fontFace: "Calibri",
      lineSpacingMultiple: 1.15
    });

    s.addText("DDA — Dynamic Difficulty Adjustment", {
      x: 0.6, y: 3.6, w: 5.7, h: 0.45,
      fontSize: 14, italic: true, color: ACCENT, fontFace: "Calibri"
    });

    s.addText([
      { text: "Верещака Богдан, гр. ПП-44", options: { breakLine: true } },
      { text: "Керівник БКР: асист. каф. САП Патерега Ю. І.", options: { breakLine: true } },
      { text: "Національний університет «Львівська політехніка», 2025" }
    ], {
      x: 0.6, y: 4.4, w: 5.7, h: 0.95,
      fontSize: 10.5, color: "8899BB", fontFace: "Calibri"
    });

    // Right panel label
    s.addText("Unity · C# · ML-Agents\nRandom Forest · RL", {
      x: 6.7, y: 2.2, w: 3.1, h: 1.2,
      fontSize: 13, color: ACCENT, fontFace: "Calibri", bold: true, align: "center"
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 2 — АКТУАЛЬНІСТЬ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };

    // Top accent strip
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT }, line: { color: ACCENT } });

    s.addText("Актуальність", {
      x: 0.5, y: 0.18, w: 9, h: 0.6,
      fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    // Big stat callouts
    const stats = [
      { val: "~40%", label: "гравців залишають гру\nчерез невідповідну складність" },
      { val: "2+", label: "десятиліття досліджень\nDDA у науковій спільноті" },
      { val: "4", label: "парадигми адаптації\nдосліджуються в проекті" },
    ];
    stats.forEach((st, i) => {
      const x = 0.5 + i * 3.15;
      s.addShape(pres.shapes.RECTANGLE, { x, y: 1.0, w: 2.85, h: 1.85,
        fill: { color: CARD_BG }, line: { color: "E2E8F0", width: 1 }, shadow: makeShadow() });
      s.addShape(pres.shapes.RECTANGLE, { x, y: 1.0, w: 2.85, h: 0.07,
        fill: { color: ACCENT }, line: { color: ACCENT } });
      s.addText(st.val, { x, y: 1.15, w: 2.85, h: 0.75,
        fontSize: 36, bold: true, color: ACCENT2, fontFace: "Calibri", align: "center" });
      s.addText(st.label, { x, y: 1.9, w: 2.85, h: 0.8,
        fontSize: 11, color: TEXT_MID, fontFace: "Calibri", align: "center" });
    });

    s.addText("Проблема", {
      x: 0.5, y: 3.05, w: 9, h: 0.38,
      fontSize: 13, bold: true, color: ACCENT2, fontFace: "Calibri"
    });
    s.addText(
      "Статична складність у комп'ютерних іграх не враховує різноманітність гравців: їхні " +
      "навички, темп навчання і рівень фрустрації змінюються від гравця до гравця і навіть " +
      "протягом однієї сесії. Єдиний фіксований рівень складності не може задовольнити всю аудиторію.",
      { x: 0.5, y: 3.4, w: 9, h: 1.1,
        fontSize: 12.5, color: TEXT_DARK, fontFace: "Calibri", lineSpacingMultiple: 1.3 }
    );

    s.addText("Рішення — DDA: автоматична адаптація параметрів гри в реальному часі на основі поведінкових метрик гравця.", {
      x: 0.5, y: 4.55, w: 9, h: 0.7,
      fontSize: 12.5, bold: true, color: "1A5276", fontFace: "Calibri",
      fill: { color: "D6EAF8" }, margin: 8
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 3 — МЕТА І ЗАВДАННЯ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT2 }, line: { color: ACCENT2 } });

    s.addText("Мета та завдання практики", {
      x: 0.5, y: 0.18, w: 9, h: 0.6,
      fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    // Goal box
    s.addShape(pres.shapes.RECTANGLE, { x: 0.5, y: 0.9, w: 9, h: 0.88,
      fill: { color: "EDE7F6" }, line: { color: ACCENT2, width: 1.5 }, shadow: makeShadow() });
    s.addText("Мета: розроблення та дослідження евристичної системи DDA для 3D-платформера на Unity, " +
      "що аналізує поведінкові метрики гравця і непомітно адаптує параметри перешкод для підтримки стану «потоку».", {
      x: 0.65, y: 0.95, w: 8.7, h: 0.78,
      fontSize: 12, color: TEXT_DARK, fontFace: "Calibri", lineSpacingMultiple: 1.25
    });

    // Task rows
    const tasks = [
      "Рефакторинг системи реєстрації смертей: структура DeathRecord, ізоляція статистики між рівнями",
      "Реалізація DDA-менеджера: кластеризація смертей, центроїд, зважений вплив за відстанню",
      "Рефакторинг усіх перешкод під інтерфейс IDynamicObstacle з базовим класом DynamicObstacleBase",
      "Логіка ускладнення при домінуванні: чекпоінти, ліміти Min/Max, індивідуальна вага (Weight)",
      "Перехід до 3D-персонажа: PlayerMovement, LastSafePosition, Animator, TransitionManager",
      "ScriptableObject-архітектура аудіосистеми: AudioEvent, SurfaceLibrary, MusicTrack",
    ];
    tasks.forEach((task, i) => {
      const y = 1.95 + i * 0.56;
      s.addShape(pres.shapes.OVAL, { x: 0.5, y: y + 0.1, w: 0.28, h: 0.28,
        fill: { color: ACCENT2 }, line: { color: ACCENT2 } });
      s.addText(String(i + 1), { x: 0.5, y: y + 0.1, w: 0.28, h: 0.28,
        fontSize: 9, bold: true, color: "FFFFFF", fontFace: "Calibri", align: "center", valign: "middle", margin: 0 });
      s.addText(task, { x: 0.95, y, w: 8.6, h: 0.52,
        fontSize: 11.5, color: TEXT_DARK, fontFace: "Calibri", valign: "middle" });
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 4 — АРХІТЕКТУРА СИСТЕМИ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT }, line: { color: ACCENT } });

    s.addText("Архітектура DDA-системи", {
      x: 0.5, y: 0.18, w: 9, h: 0.6,
      fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    // Left column – components
    const comps = [
      { title: "PlayerTracker", desc: "LastSafePosition, трекінг позиції щокадру при контакті з поверхнею" },
      { title: "KillPlane", desc: "Передає усереднену координату смерті (Lerp між LastSafePos і точкою падіння)" },
      { title: "GameSession", desc: "DeathRecord list, деathWindow фільтрація, генерація подій OnDeathCluster / OnPlayerDominating" },
      { title: "DDAManager", desc: "Registry Pattern, центроїд кластера, зважений Mathf.Lerp за відстанню, AdjustDifficulty" },
    ];
    comps.forEach((c, i) => {
      const y = 1.0 + i * 1.05;
      s.addShape(pres.shapes.RECTANGLE, { x: 0.4, y, w: 4.3, h: 0.92,
        fill: { color: CARD_BG }, line: { color: "CBD5E1", width: 1 }, shadow: makeShadow() });
      s.addShape(pres.shapes.RECTANGLE, { x: 0.4, y, w: 0.06, h: 0.92, fill: { color: ACCENT }, line: { color: ACCENT } });
      s.addText(c.title, { x: 0.55, y: y + 0.05, w: 4.1, h: 0.32, fontSize: 12, bold: true, color: TEXT_DARK, fontFace: "Calibri" });
      s.addText(c.desc, { x: 0.55, y: y + 0.36, w: 4.1, h: 0.52, fontSize: 10, color: TEXT_MID, fontFace: "Calibri" });
    });

    // Right column – obstacle types
    s.addText("Типи перешкод  ·  IDynamicObstacle", {
      x: 5.2, y: 1.0, w: 4.5, h: 0.38, fontSize: 12, bold: true, color: ACCENT2, fontFace: "Calibri"
    });

    const obs = [
      { name: "FallingPlatform", detail: "шкала shakingTime, Min/Max" },
      { name: "MovingPlatform", detail: "швидкість ÷ multiplier, Min/Max" },
      { name: "SpikyPlatform", detail: "затримка обертання, Min/Max" },
      { name: "SimpleDynamicPlatform", detail: "масштаб X/Z, захист Y" },
    ];
    obs.forEach((o, i) => {
      const y = 1.5 + i * 0.88;
      s.addShape(pres.shapes.RECTANGLE, { x: 5.2, y, w: 4.5, h: 0.75,
        fill: { color: "F0FDF4" }, line: { color: "BBF7D0", width: 1 } });
      s.addText(o.name, { x: 5.35, y: y + 0.06, w: 4.2, h: 0.3, fontSize: 11.5, bold: true, color: "166534", fontFace: "Calibri" });
      s.addText(o.detail, { x: 5.35, y: y + 0.36, w: 4.2, h: 0.3, fontSize: 10.5, color: TEXT_MID, fontFace: "Calibri" });
    });

    // TransitionManager note
    s.addShape(pres.shapes.RECTANGLE, { x: 5.2, y: 5.0, w: 4.5, h: 0.45,
      fill: { color: "FFF9C4" }, line: { color: "F9A825", width: 1 } });
    s.addText("TransitionManager: зміни видимі лише під затемнення екрану", {
      x: 5.3, y: 5.02, w: 4.3, h: 0.4, fontSize: 10, italic: true, color: "7B5200", fontFace: "Calibri"
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 5 — АЛГОРИТМ КЛАСТЕРИЗАЦІЇ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT2 }, line: { color: ACCENT2 } });

    s.addText("Алгоритм евристичної DDA", {
      x: 0.5, y: 0.18, w: 9, h: 0.6, fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    // Flow steps
    const steps = [
      { n: "1", title: "Реєстрація смерті", body: "KillPlane передає Vector3.Lerp(lastSafePos, fallPos, 0.5f) → DeathRecord { position, timestamp }" },
      { n: "2", title: "Фільтрація вікна", body: "recentDeaths.RemoveAll(d => Time.time − d.timestamp > deathWindow)\nВидаляє застарілі записи, запобігає помилковим кластерам" },
      { n: "3", title: "Перевірка кластера", body: "if (recentDeaths.Count >= deathThreshold) → GameEvents.OnDeathClusterDetected(centroid)\ncentroid = середнє арифметичне позицій смертей" },
      { n: "4", title: "Зважений вплив", body: "normalizedDist = dist / adaptationRadius\neffectiveMultiplier = Mathf.Lerp(easingFactor, 1.0f, normalizedDist)" },
      { n: "5", title: "Застосування", body: "ApplyDDA(effectiveMultiplier) для кожної IDynamicObstacle в радіусі\nЗахист: нове значення застосовується лише якщо воно сильніше за поточне" },
    ];

    steps.forEach((st, i) => {
      const x = 0.35 + i * 1.88;
      s.addShape(pres.shapes.RECTANGLE, { x, y: 1.05, w: 1.72, h: 3.85,
        fill: { color: CARD_BG }, line: { color: "E2E8F0", width: 1 }, shadow: makeShadow() });
      s.addShape(pres.shapes.OVAL, { x: x + 0.62, y: 1.05, w: 0.48, h: 0.48,
        fill: { color: ACCENT2 }, line: { color: ACCENT2 } });
      s.addText(st.n, { x: x + 0.62, y: 1.05, w: 0.48, h: 0.48,
        fontSize: 14, bold: true, color: "FFFFFF", fontFace: "Calibri", align: "center", valign: "middle", margin: 0 });
      s.addText(st.title, { x: x + 0.06, y: 1.65, w: 1.6, h: 0.55,
        fontSize: 10.5, bold: true, color: TEXT_DARK, fontFace: "Calibri", align: "center" });
      s.addText(st.body, { x: x + 0.06, y: 2.22, w: 1.6, h: 2.6,
        fontSize: 9.5, color: TEXT_MID, fontFace: "Calibri", lineSpacingMultiple: 1.2 });

      if (i < 4) {
        s.addShape(pres.shapes.LINE, {
          x: x + 1.72, y: 2.95, w: 0.16, h: 0,
          line: { color: ACCENT, width: 2 }
        });
        s.addText("›", { x: x + 1.82, y: 2.78, w: 0.2, h: 0.34,
          fontSize: 16, bold: true, color: ACCENT, fontFace: "Calibri", margin: 0 });
      }
    });

    s.addShape(pres.shapes.RECTANGLE, { x: 0.35, y: 5.1, w: 9.3, h: 0.38,
      fill: { color: "E0F2FE" }, line: { color: ACCENT, width: 1 } });
    s.addText("Ускладнення: при проходженні чекпоінта без смертей → OnPlayerDominating → complicationFactor=1.15 до перешкод попереду без fade", {
      x: 0.45, y: 5.12, w: 9.1, h: 0.35,
      fontSize: 10, color: "075985", fontFace: "Calibri", italic: true
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 6 — ТЕХНОЛОГІЧНИЙ СТЕК
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT }, line: { color: ACCENT } });

    s.addText("Технологічний стек", {
      x: 0.5, y: 0.18, w: 9, h: 0.6, fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    const techs = [
      { name: "Unity Engine", role: "Ігровий рушій", note: "Фізика, рендер, аудіо, сцени" },
      { name: "C# / OOP", role: "Ігрова логіка", note: "Інтерфейси, патерни Registry, ScriptableObject" },
      { name: "Input System", role: "Управління вводом", note: "Гнучкий контролер гравця" },
      { name: "Cinemachine", role: "Камери", note: "Плавні переходи, Virtual Camera" },
      { name: "ProBuilder", role: "Левел-дизайн", note: "Швидке прототипування геометрії" },
      { name: "ScriptableObject Audio", role: "Аудіосистема", note: "AudioEvent, SurfaceLibrary, MusicTrack" },
      { name: "Unity ML-Agents", role: "Reinforcement Learning", note: "Agent + Director, Self-Play" },
      { name: "ONNX / Unity Sentis", role: "ML-інференс", note: "Запуск Random Forest в Unity" },
    ];

    techs.forEach((t, i) => {
      const col = i % 4;
      const row = Math.floor(i / 4);
      const x = 0.4 + col * 2.32;
      const y = 1.0 + row * 2.0;
      s.addShape(pres.shapes.RECTANGLE, { x, y, w: 2.15, h: 1.75,
        fill: { color: CARD_BG }, line: { color: "E2E8F0", width: 1 }, shadow: makeShadow() });
      s.addShape(pres.shapes.RECTANGLE, { x, y, w: 2.15, h: 0.07,
        fill: { color: i < 4 ? ACCENT : ACCENT2 }, line: { color: i < 4 ? ACCENT : ACCENT2 } });
      s.addText(t.name, { x: x + 0.08, y: y + 0.14, w: 2.0, h: 0.48,
        fontSize: 11, bold: true, color: TEXT_DARK, fontFace: "Calibri" });
      s.addText(t.role, { x: x + 0.08, y: y + 0.62, w: 2.0, h: 0.38,
        fontSize: 9.5, color: i < 4 ? ACCENT : ACCENT2, fontFace: "Calibri", bold: true });
      s.addText(t.note, { x: x + 0.08, y: y + 1.0, w: 2.0, h: 0.6,
        fontSize: 9, color: TEXT_MID, fontFace: "Calibri" });
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 7 — АНАЛІТИКА: МЕТРИКИ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT2 }, line: { color: ACCENT2 } });

    s.addText("Система аналітики та метрики", {
      x: 0.5, y: 0.18, w: 9, h: 0.6, fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    // Left: input metrics
    s.addText("Збір вхідних даних", { x: 0.5, y: 0.92, w: 4.4, h: 0.42,
      fontSize: 13, bold: true, color: ACCENT2, fontFace: "Calibri" });

    const inputs = [
      "Координати смерті + LastSafePosition",
      "Часові мітки → DeathRecord (time window)",
      "Частота натискань клавіш (APM)",
      "Бездіяльність після смерті (Idle Time)",
      "Успішність взаємодії з перешкодами",
      "Активація чекпоінтів, час рівня",
    ];
    inputs.forEach((t, i) => {
      s.addShape(pres.shapes.RECTANGLE, { x: 0.5, y: 1.42 + i * 0.57, w: 4.4, h: 0.5,
        fill: { color: i % 2 === 0 ? "F8FAFF" : CARD_BG }, line: { color: "E2E8F0", width: 1 } });
      s.addText(t, { x: 0.65, y: 1.46 + i * 0.57, w: 4.1, h: 0.42,
        fontSize: 11, color: TEXT_DARK, fontFace: "Calibri", valign: "middle" });
    });

    // Right: output data
    s.addText("Результати аналітики (NDJSON)", { x: 5.2, y: 0.92, w: 4.4, h: 0.42,
      fontSize: 13, bold: true, color: "027A48", fontFace: "Calibri" });

    const outputs = [
      { evt: "death", desc: "координата, причина, спроба №" },
      { evt: "death_cluster", desc: "центроїд, радіус, кількість смертей" },
      { evt: "obstacle_result", desc: "спроби, успіхи, % прохідності" },
      { evt: "level_start / end", desc: "час проходження, к-ть смертей" },
      { evt: "checkpoint", desc: "позиція, деathsSinceLastCheckpoint" },
      { evt: "input_rate", desc: "APM, тривалість натискань кнопок" },
    ];
    outputs.forEach((o, i) => {
      const y = 1.42 + i * 0.57;
      s.addShape(pres.shapes.RECTANGLE, { x: 5.2, y, w: 4.4, h: 0.5,
        fill: { color: i % 2 === 0 ? "F0FDF4" : CARD_BG }, line: { color: "BBF7D0", width: 1 } });
      s.addText(o.evt, { x: 5.3, y: y + 0.04, w: 1.6, h: 0.42,
        fontSize: 10, bold: true, color: "166534", fontFace: "Calibri", valign: "middle" });
      s.addText(o.desc, { x: 6.95, y: y + 0.04, w: 2.6, h: 0.42,
        fontSize: 10, color: TEXT_MID, fontFace: "Calibri", valign: "middle" });
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 8 — ПЛАН ПОРІВНЯЛЬНОГО ДОСЛІДЖЕННЯ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT }, line: { color: ACCENT } });

    s.addText("Дорожня карта БКР: 4 підходи до DDA", {
      x: 0.5, y: 0.18, w: 9, h: 0.6, fontSize: 24, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    const stages = [
      {
        num: "01", status: "ВИКОНАНО", color: "166534", bg: "DCFCE7", border: "86EFAC",
        title: "Статична база",
        points: ["Без DDA — контрольна група для A/B тестів", "Система аналітики NDJSON", "Базовий ігровий прототип"]
      },
      {
        num: "02", status: "ВИКОНАНО", color: "166534", bg: "DCFCE7", border: "86EFAC",
        title: "Евристична DDA",
        points: ["DeathRecord + кластеризація", "Зважений вплив за відстанню", "TransitionManager + ускладнення"]
      },
      {
        num: "03", status: "В ПЛАНІ", color: "0369A1", bg: "DBEAFE", border: "93C5FD",
        title: "Random Forest",
        points: ["Датасет із плейтестів", "Python + scikit-learn навчання", "Інтеграція через ONNX / Unity Sentis"]
      },
      {
        num: "04", status: "В ПЛАНІ", color: "6D28D9", bg: "EDE9FE", border: "C4B5FD",
        title: "Reinforcement Learning",
        points: ["Unity ML-Agents Toolkit", "Бот-гравець + Режисер (Self-Play)", "Asymmetric Adversarial Training"]
      },
    ];

    stages.forEach((st, i) => {
      const x = 0.4 + i * 2.32;
      s.addShape(pres.shapes.RECTANGLE, { x, y: 1.05, w: 2.15, h: 3.9,
        fill: { color: st.bg }, line: { color: st.border, width: 1.5 }, shadow: makeShadow() });

      s.addShape(pres.shapes.OVAL, { x: x + 0.72, y: 1.05, w: 0.7, h: 0.7,
        fill: { color: i < 2 ? "166534" : (i === 2 ? "0369A1" : "6D28D9") }, line: { color: "transparent" } });
      s.addText(st.num, { x: x + 0.72, y: 1.05, w: 0.7, h: 0.7,
        fontSize: 16, bold: true, color: "FFFFFF", fontFace: "Calibri", align: "center", valign: "middle", margin: 0 });

      s.addShape(pres.shapes.RECTANGLE, { x: x + 0.3, y: 1.84, w: 1.55, h: 0.32,
        fill: { color: i < 2 ? "166534" : (i === 2 ? "0369A1" : "6D28D9") }, line: { color: "transparent" } });
      s.addText(st.status, { x: x + 0.3, y: 1.85, w: 1.55, h: 0.3,
        fontSize: 8.5, bold: true, color: "FFFFFF", fontFace: "Calibri", align: "center", valign: "middle", margin: 0 });

      s.addText(st.title, { x: x + 0.08, y: 2.25, w: 2.0, h: 0.5,
        fontSize: 11.5, bold: true, color: st.color, fontFace: "Calibri", align: "center" });

      st.points.forEach((pt, j) => {
        s.addText([{ text: pt, options: { bullet: true } }], {
          x: x + 0.1, y: 2.82 + j * 0.62, w: 2.0, h: 0.57,
          fontSize: 9.5, color: TEXT_DARK, fontFace: "Calibri"
        });
      });
    });

    s.addText("Порівняння за метриками: Completion Rate · Session Time · Непомітність змін для гравця", {
      x: 0.5, y: 5.1, w: 9, h: 0.38,
      fontSize: 11, italic: true, color: TEXT_MID, fontFace: "Calibri", align: "center"
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 9 — РЕЗУЛЬТАТИ ПРАКТИКИ
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_LIGHT };
    s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.08, fill: { color: ACCENT }, line: { color: ACCENT } });

    s.addText("Результати практики", {
      x: 0.5, y: 0.18, w: 9, h: 0.6, fontSize: 26, bold: true, color: TEXT_DARK, fontFace: "Calibri"
    });

    const results = [
      {
        title: "Евристична DDA", icon: "✓",
        items: ["Кластеризація смертей (DeathRecord)", "Зважений вплив, захист параметрів", "Ускладнення при домінуванні"]
      },
      {
        title: "Архітектура", icon: "✓",
        items: ["Registry Pattern (DDAManager)", "IDynamicObstacle + DynamicObstacleBase", "4 типи адаптивних перешкод"]
      },
      {
        title: "Гравець і світ", icon: "✓",
        items: ["3D-модель, Animator, Quaternion.Slerp", "LastSafePosition у PlayerTracker", "TransitionManager (fade in/out)"]
      },
      {
        title: "Аудіо і UI", icon: "✓",
        items: ["ScriptableObject: AudioEvent, Surface-Library", "Рандомізована гучність і тон", "UI: смерті, таймер рівня"]
      },
    ];

    results.forEach((r, i) => {
      const x = 0.4 + (i % 2) * 4.8;
      const y = 1.0 + Math.floor(i / 2) * 2.3;
      s.addShape(pres.shapes.RECTANGLE, { x, y, w: 4.4, h: 2.1,
        fill: { color: CARD_BG }, line: { color: "E2E8F0", width: 1 }, shadow: makeShadow() });
      s.addShape(pres.shapes.RECTANGLE, { x, y, w: 4.4, h: 0.07,
        fill: { color: i < 2 ? ACCENT : ACCENT2 }, line: { color: i < 2 ? ACCENT : ACCENT2 } });
      s.addText("✓", { x: x + 0.1, y: y + 0.12, w: 0.35, h: 0.35,
        fontSize: 13, bold: true, color: i < 2 ? ACCENT : ACCENT2, fontFace: "Calibri" });
      s.addText(r.title, { x: x + 0.45, y: y + 0.1, w: 3.8, h: 0.38,
        fontSize: 13, bold: true, color: TEXT_DARK, fontFace: "Calibri" });
      r.items.forEach((item, j) => {
        s.addText([{ text: item, options: { bullet: true } }], {
          x: x + 0.15, y: y + 0.55 + j * 0.47, w: 4.1, h: 0.44,
          fontSize: 11, color: TEXT_DARK, fontFace: "Calibri"
        });
      });
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // SLIDE 10 — ВИСНОВКИ (dark)
  // ═══════════════════════════════════════════════════════════════════════════
  {
    const s = pres.addSlide();
    s.background = { color: BG_DARK };

    s.addShape(pres.shapes.RECTANGLE, { x: 9.65, y: 0, w: 0.35, h: 5.625, fill: { color: ACCENT }, line: { color: ACCENT } });

    s.addText("Висновки та подальші кроки", {
      x: 0.5, y: 0.35, w: 9, h: 0.65,
      fontSize: 28, bold: true, color: TEXT_LIGHT, fontFace: "Calibri"
    });

    const conclusions = [
      "Евристична DDA-система повністю реалізована і протестована: кластеризація смертей, зважений вплив, ускладнення при домінуванні.",
      "Архітектура проекту свідомо орієнтована на масштабованість — заміна евристики на ML відбудеться без змін у коді перешкод.",
      "Аналітична підсистема формує структуровані NDJSON-логи для навчання майбутніх ML-моделей.",
      "Концепція Asymmetric Self-Play (бот-гравець + бот-режисер) забезпечить мільйони симуляцій для RL без участі людей.",
    ];
    conclusions.forEach((c, i) => {
      s.addShape(pres.shapes.OVAL, { x: 0.5, y: 1.2 + i * 0.88, w: 0.36, h: 0.36,
        fill: { color: ACCENT }, line: { color: ACCENT } });
      s.addText(String(i + 1), { x: 0.5, y: 1.2 + i * 0.88, w: 0.36, h: 0.36,
        fontSize: 11, bold: true, color: BG_DARK, fontFace: "Calibri", align: "center", valign: "middle", margin: 0 });
      s.addText(c, { x: 0.98, y: 1.15 + i * 0.88, w: 8.5, h: 0.82,
        fontSize: 12.5, color: TEXT_LIGHT, fontFace: "Calibri", lineSpacingMultiple: 1.2 });
    });

    s.addShape(pres.shapes.RECTANGLE, { x: 0.5, y: 4.88, w: 9.0, h: 0.5,
      fill: { color: ACCENT2, transparency: 10 }, line: { color: ACCENT2 } });
    s.addText("Наступний крок: плейтести → датасет → Random Forest (scikit-learn + ONNX) → ML-Agents RL", {
      x: 0.6, y: 4.9, w: 8.8, h: 0.46,
      fontSize: 11.5, bold: true, color: "FFFFFF", fontFace: "Calibri", align: "center"
    });
  }

  await pres.writeFile({ fileName: "/mnt/user-data/outputs/DDA_Presentation.pptx" });
  console.log("Done!");
}

main().catch(console.error);

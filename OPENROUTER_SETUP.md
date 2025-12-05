# OpenRouter API Setup - Echtes Natural Language Processing

## Schritt 1: Account erstellen

1. Gehe zu: https://openrouter.ai/
2. Klicke auf "Sign In" (oben rechts)
3. Login mit Google oder GitHub
4. **KOSTENLOS** - keine Kreditkarte nötig für Start

## Schritt 2: Kostenloses Guthaben

OpenRouter gibt **$1-5 kostenloses Guthaben** zum Testen!

**Verbrauch:**
- Claude 3.5 Sonnet: ~$0.003 per Request
- **Mit $1 = ~333 Commands**
- **Mit $5 = ~1666 Commands**

Das reicht erstmal zum Testen!

## Schritt 3: API Key generieren

1. Gehe zu: https://openrouter.ai/keys
2. Klicke "Create Key"
3. Name: "MeArm Control"
4. Kopiere den Key (Format: `sk-or-v1-...`)

## Schritt 4: API Key in System setzen

```bash
# Temporär (nur aktuelle Session):
export OPENROUTER_API_KEY="sk-or-v1-DEIN_KEY_HIER"

# Permanent (in .bashrc):
echo 'export OPENROUTER_API_KEY="sk-or-v1-DEIN_KEY_HIER"' >> ~/.bashrc
source ~/.bashrc
```

## Schritt 5: HTTP Bridge neu starten

```bash
# Stoppe aktuellen Server (Ctrl+C im Terminal)
# Oder:
pkill -f http_bridge.py

# Neu starten:
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/ros_mcp_server_v2/ros_mcp_server_v2/http_bridge.py
```

## Schritt 6: Testen!

### Test 1: Status prüfen
```bash
curl http://localhost:8000/status | jq
```

**Erwartete Ausgabe:**
```json
{
  "ros2_initialized": true,
  "llm_available": true,    ← Sollte jetzt true sein!
  "ready": true
}
```

### Test 2: Echte Natural Language Commands

```bash
# Deutsch
curl -X POST http://localhost:8000/command \
  -H "Content-Type: application/json" \
  -d '{"text": "drehe dich ein bisschen nach links"}' | jq

# Englisch
curl -X POST http://localhost:8000/command \
  -H "Content-Type: application/json" \
  -d '{"text": "turn the arm a little bit to the right"}' | jq

# Umgangssprachlich
curl -X POST http://localhost:8000/command \
  -H "Content-Type: application/json" \
  -d '{"text": "move back to the beginning"}' | jq
```

**Erwartete Ausgabe:**
```json
{
  "status": "success",
  "input": "drehe dich ein bisschen nach links",
  "parsed": {
    "angle": 0.5,
    "reasoning": "small rotation to the left, about 30 degrees",
    "method": "llm"    ← LLM wurde genutzt!
  },
  "executed": true
}
```

### Test 3: Web Interface

Öffne http://localhost:8080 und teste:
- "dreh dich mal 20 grad"
- "go back to zero"
- "rotate approximately 45 degrees left"

Alle sollten funktionieren!

## Was der LLM versteht:

### Relationale Commands:
- "ein bisschen", "a little bit" → ~15-30°
- "halb", "halfway" → 45°
- "ganz", "fully" → 90°

### Umgangssprachlich:
- "dreh dich mal" = "rotate"
- "geh zurück" = "reset"
- "move to the beginning" = "reset"

### Mehrsprachig:
- Deutsch ✅
- Englisch ✅
- (Theoretisch auch andere Sprachen)

## Kosten-Übersicht

| Nutzung | Commands | Kosten |
|---------|----------|--------|
| Testing (erste Woche) | ~100 | $0.30 |
| Tägliche Nutzung | ~50/Tag | $0.15/Tag |
| Intensive Entwicklung | ~500/Tag | $1.50/Tag |

**Mit $5 Free Credit = ~1-2 Monate tägliche Nutzung!**

## Troubleshooting

### "llm_available": false

```bash
# Prüfe, ob Key gesetzt ist:
echo $OPENROUTER_API_KEY

# Falls leer, neu setzen:
export OPENROUTER_API_KEY="sk-or-v1-..."
```

### LLM Request fehlschlägt

Logs prüfen (Terminal wo http_bridge.py läuft):
```
⚠ LLM failed, using fallback: <Error>
```

Mögliche Fehler:
- Kein Guthaben mehr → Aufladen bei OpenRouter
- Ungültiger Key → Neu generieren
- Netzwerkproblem → Internetverbindung prüfen

**Fallback:** System nutzt automatisch Regex Parser wenn LLM fehlschlägt!

---

## Nächster Schritt: MeArm Hardware

Siehe: `MEARM_HARDWARE_SETUP.md`

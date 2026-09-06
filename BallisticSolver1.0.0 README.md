# .338NM Ballistic Solver

### 1 · Backend

```bash
cd backend
python -m venv .venv && source .venv/bin/activate      # optional
pip install -r requirements.txt
uvicorn main:app --reload --port 8000
```

Verify: `curl http://localhost:8000/api/health` → `{"status":"ok"}`.

### 2 · Frontend

```bash
cd frontend
npm install
npm run dev
```

Open **http://localhost:5173**. The dev server proxies `/api` to the backend on
:8000, so everything runs from one origin.

### Tests

```bash
cd backend
python -m pytest ballistics/tests/ -q
```

## structure

```
backend/
  main.py                  
  requirements.txt
  ballistics/
    drag_tables.py         
    atmosphere.py          
    stability.py           
    solver.py              
    bullets.py             
    tests/test_solver.py   
frontend/
  src/
    App.jsx                
    ControlPanel.jsx       
    TrajectoryHUD.jsx      
    RangeCard.jsx          
    api.js                 
    styles.css             
  vite.config.js           dev proxy -> :8000
```
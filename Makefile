.PHONY: up down logs test frontend-dev backend-dev simulator-dev

up:
	docker compose up -d

down:
	docker compose down

logs:
	docker compose logs -f --tail=100

test:
	cd src/apps/center-frontend && npm test
	cd src/apps/center-backend && pytest
	cd demo/robot-simulator && PYTHONPATH=. pytest

frontend-dev:
	cd src/apps/center-frontend && npm run dev

backend-dev:
	cd src/apps/center-backend && uvicorn app.main:app --reload

simulator-dev:
	cd demo/robot-simulator && python -m simulator.main

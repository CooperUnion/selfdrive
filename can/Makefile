.ONESHELL:

out_dir = ./
generator = gen_dbc.py

.PHONY default:
default: igvc_can.dbc

.venv: requirements.txt
	test -d .venv || python3 -m venv .venv
	. .venv/bin/activate
	pip install --upgrade pip
	pip install -Ur requirements.txt
	touch .venv/touched

.PHONY activate_venv:
activate_venv: .venv
	@. .venv/bin/activate

igvc_can.dbc: gen_dbc.py can.yml | activate_venv
	python3 $(generator)

clean:
	rm -rf .venv
	find -iname "*.pyc" -delete

lint: $(generator) activate_venv
	flake8 $(generator)
	black $(generator)

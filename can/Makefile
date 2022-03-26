.ONESHELL:

out_dir = ./
generator = gen_dbc.py

cantools_match = $(shell find cantools -type f | grep -v cantools/build/)

.PHONY default:
default: igvc_can.dbc

.PHONY dbw_files:
dbw_files: can_gen.c can_gen.h

.venv/touched: requirements.txt $(cantools_match)
	test -d .venv || python3 -m venv .venv
	. .venv/bin/activate
	pip install --upgrade pip
	pip install -Ur requirements.txt
	pip install cantools/
	touch .venv/touched

igvc_can.dbc: gen_dbc.py can.yml .venv/touched
	. .venv/bin/activate
	python3 $(generator)

can_gen.c can_gen.h : igvc_can.dbc .venv/touched
	. .venv/bin/activate
	cantools generate_c_source igvc_can.dbc --database-name CAN --use-float
	mv CAN.c can_gen.c
	mv CAN.h can_gen.h
	sed -i 's\CAN_H\CAN_GEN_H\g' can_gen.h
	sed -i 's\CAN.h\can_gen.h\g' can_gen.c
	cp can_gen.* ../dbw/node_fw/src/io/

clean:
	rm -rf .venv
	find -iname "*.pyc" -delete

lint: $(generator) .venv/touched
	. .venv/bin/activate
	flake8 $(generator)
	black $(generator)



setup: .venv	
	. .venv/bin/activate; pip3 install -Ur requirements.txt

.venv:
	python3 -m venv .venv

pixel_mapping:
	. .venv/bin/activate; python3 ./pixel_mapping.py
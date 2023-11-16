.venv:
	python -m venv .venv &&\
	source .venv/bin/activate &&\
	pip install -r requirements.txt

pixel_mapping: .venv
	. .venv/bin/activate; python3 ./pixel_mapping.py

protobuf:
	protoc --python_out=. ./command_schemas.proto ./nanopb.proto
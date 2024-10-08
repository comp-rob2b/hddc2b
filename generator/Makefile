shacl:
	@pyshacl_validate -s hddc2b.shacl.ttl -m -f human example.hddc2b.json

schema:
	@check-jsonschema --schemafile hddc2b.schema.json example.hddc2b.json

jinja:
	@j2 -f json hdr.jinja example.hddc2b.json
	@j2 -f json impl.jinja example.hddc2b.json
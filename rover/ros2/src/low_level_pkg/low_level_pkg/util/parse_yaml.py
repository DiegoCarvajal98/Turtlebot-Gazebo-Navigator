from collections import defaultdict
import yaml


def parse_preserving_duplicates(src):
    # We deliberately define a fresh class inside the function,
    # because add_constructor is a class method and we don't want to
    # mutate pyyaml classes.
    class PreserveDuplicatesLoader(yaml.loader.Loader):
        pass

    def map_constructor(loader, node, deep=False):
        """Walk the mapping, recording any duplicate keys.

        """
        mapping = defaultdict(list)
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            value = loader.construct_object(value_node, deep=deep)

            mapping[key].append(value)

        return mapping

    PreserveDuplicatesLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, map_constructor)
    return yaml.load(src, PreserveDuplicatesLoader)
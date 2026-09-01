from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class CaseLayout:
    name: str
    mode: str
    test_dir: Path
    bag: Path
    ulog: Path | None
    dynamic_enabled: bool

    def to_dict(self) -> dict[str, object]:
        values = asdict(self)
        values["test_dir"] = str(self.test_dir)
        values["bag"] = str(self.bag)
        values["ulog"] = str(self.ulog) if self.ulog else None
        return values


def parse_key_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def find_ulog(root: Path, name: str, mode: str) -> Path | None:
    candidates = sorted(root.glob("*.ulg"))
    search_terms = (name.lower(), mode.lower())
    for term in search_terms:
        for candidate in candidates:
            if term in candidate.stem.lower():
                return candidate
    return None


def discover_cases(root: Path) -> list[CaseLayout]:
    cases: list[CaseLayout] = []
    for notes_path in sorted(root.rglob("test_notes.txt")):
        notes = parse_key_values(notes_path)
        name = notes.get("TEST_ID", notes_path.parent.name)
        mode = notes.get("RUN_MODE", "unknown").lower()
        bag_files = sorted((notes_path.parent / "rosbag").glob("*.mcap"))
        if not bag_files:
            continue
        cases.append(CaseLayout(
            name=name,
            mode=mode,
            test_dir=notes_path.parent,
            bag=bag_files[0],
            ulog=find_ulog(root, name, mode),
            dynamic_enabled=(
                notes.get("SUPER_LIO_DYNAMIC_ENABLE", "false").lower() == "true"
            ),
        ))
    order = {"baseline": 0, "shadow": 1}
    return sorted(cases, key=lambda case: (order.get(case.mode, 99), case.name))

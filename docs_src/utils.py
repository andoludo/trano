import ast
import os
from pathlib import Path
from typing import Any, List, Optional

from dotenv import load_dotenv
from openai import OpenAI
from pydantic import BaseModel

load_dotenv()


class Context(BaseModel):
    question: str
    answer: Optional[str] = None
    object: str


class BeforeAfterContext(BaseModel):
    before: Context
    after: Context


class Configuration(BaseModel):
    path: Optional[Path] = None
    object_before: Optional[str] = None
    object_after: Optional[str] = None
    question_before: str
    question_after: str

    def get_contexts(self) -> BeforeAfterContext:
        contexts = [
            Context(
                object=self.object_before
                if self.object_before is not None
                else self.path.read_text(),
                question=self.question_before,
            ),
            Context(
                object=self.object_after
                if self.object_after is not None
                else self.path.read_text(),
                question=self.question_after,
            ),
        ]
        contexts = [get_context(c) for c in contexts]
        return BeforeAfterContext(before=contexts[0], after=contexts[1])


class Source(BaseModel):
    title: str
    source: str
    report_path: Optional[Configuration] = None
    question: str

    def report(self) -> Optional[str]:
        if self.report_path.path:
            return self.report_path.path.read_text()
        return None


class Tutorial(BaseModel):
    title: str
    name: str
    configuration: Configuration
    code_blocks_source: list[Source]
    test_file_path: Optional[Path] = None
    destination_path: Optional[Path] = None

    def model_post_init(self, __context: Any) -> None:  # noqa: ANN401
        source_path = Path(__file__).parents[1].joinpath("tests/tutorials")
        self.configuration.path = source_path.joinpath(f"{self.name}.yaml")
        self.test_file_path = source_path.joinpath("test_tutorials.py")
        self.destination_path = (
            Path(__file__).parents[1].joinpath(f"docs/tutorials/{self.name}.md")
        )
        for code_block_source in self.code_blocks_source:
            file_ = code_block_source.source.replace("test_", "")
            path_ = source_path.joinpath(f"{file_}.html")
            if path_.exists():
                code_block_source.report_path.path = path_

    def write(self) -> None:
        self.destination_path.write_text(self.build())

    def build(self) -> str:
        configuration_paragraph = self.configuration.get_contexts()

        output = f"""
## {self.title}
{configuration_paragraph.before.answer}

```yaml title='{self.configuration_title()}'
{self.configuration.path.read_text()}
```
\n
{configuration_paragraph.after.answer}
"""
        for source, code_block in zip(
            self.code_blocks_source, self.code_blocks_with_context()
        ):
            if code_block:
                output += f"""
## {source.title}
```python title='{source.source.replace("test_", " ").replace("_", " ").capitalize()}'
{code_block.object}
```
{code_block.answer}
                """

            if source.report_path:
                report_context = source.report_path.get_contexts()
                output += f"""
## Results
{report_context.before.answer}
{source.report()}
{report_context.after.answer}
"""
        return output

    def code_blocks(self) -> List[Context]:
        return [
            Context(
                object=extract_function_snippet(self.test_file_path, code_block.source),
                question=code_block.question,
            )
            for code_block in self.code_blocks_source
        ]

    def code_blocks_with_context(self) -> List[Context]:
        contexts = [get_context(code_block) for code_block in self.code_blocks()]
        return contexts

    def configuration_title(self) -> str:
        return self.configuration.path.stem.replace("_", " ").capitalize()


def extract_function_snippet(file_path: Path, function_name: str) -> Optional[str]:
    file_content = file_path.read_text()
    tree = ast.parse(file_content)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            start_line = node.body[0].lineno
            end_line = node.body[-1].end_lineno
            with open(file_path) as f:
                lines = f.readlines()[start_line - 1: end_line]
            function_text = "".join(lines)
            return function_text
    return None


def get_context(context: Context) -> Context:
    client = OpenAI(
        api_key=os.environ.get("OPENAI_API_KEY"),
    )
    chat_completion = client.chat.completions.create(
        messages=[
            {"role": "user", "content": f"{context.question}:{context.object}"},
        ],
        model="gpt-4o-mini",
        temperature=0,  # Set temperature to 0 for more deterministic responses
    )
    context.answer = chat_completion.choices[0].message.content
    return context

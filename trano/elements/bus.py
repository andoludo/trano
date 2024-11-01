from pathlib import Path
from typing import List, Optional

import pandas as pd
from pydantic import BaseModel, Field, computed_field

from trano.elements.base import BaseElement
from trano.elements.inputs import BaseInputOutput


class ValidationData(BaseModel):
    data: Optional[str] = None
    columns: List[str] = Field([])


class DataBus(BaseElement):
    name: str
    position: Optional[List[float]] = None
    spaces: List[str]
    non_connected_ports: List[BaseInputOutput] = Field(default=[])
    external_data: Optional[Path] = None

    @computed_field
    def validation_data(self) -> ValidationData:
        if not self.external_data:
            return ValidationData()
        return transform_csv_to_table(self.external_data)


def transform_csv_to_table(
    file_path: Path, total_second: bool = True
) -> ValidationData:
    data = pd.read_csv(file_path, index_col=0, infer_datetime_format=True)
    data = data.ffill().bfill()
    data = data.dropna(axis=1)
    data.index = pd.to_datetime(data.index)
    if total_second:
        data.index = (data.index - data.first_valid_index()).total_seconds()  # type: ignore
    else:
        data.index = data.index.astype(int) // 10 ** 9
    data_str = data.to_csv(sep=",", header=False, lineterminator=";")
    if data_str.endswith(";"):
        data_str = data_str[:-1]
    return ValidationData(data=data_str, columns=data.columns.tolist())

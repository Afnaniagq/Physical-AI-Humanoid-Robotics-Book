import React from 'react';
import { useHistory } from '@docusaurus/router';

export default function StartCourseButton() {
  const history = useHistory();

  const handleClick = () => {
    history.push('/docs/intro'); // Assuming the first chapter is at /docs/intro
  };

  return (
    <button onClick={handleClick}>
      Start Course
    </button>
  );
}

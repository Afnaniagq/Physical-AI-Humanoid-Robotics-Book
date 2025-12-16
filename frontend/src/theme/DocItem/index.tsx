import React, {type ReactNode} from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type {WrapperProps} from '@docusaurus/types';
import TranslationToggle from '@site/src/components/TranslationToggle';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): ReactNode {
  return (
    <>
      <TranslationToggle />
      <DocItem {...props} />
    </>
  );
}